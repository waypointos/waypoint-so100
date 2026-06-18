package calibration

// ServoReading is one raw sample from a servo.
type ServoReading struct {
	PositionRaw uint16
	CurrentRaw  uint16
	OK          bool
}

// ServoClient is the seek loop's view of a servo, satisfied by the NATS client
// (internal/servo) in production and a fake plant in tests.
type ServoClient interface {
	SetMode(id uint32, mode uint32) error
	SetTorqueLimit(id uint32, raw uint16) error
	SetOvercurrentLimit(id uint32, raw uint16) error
	SetAngleLimits(id uint32, min, max uint16) error
	SetGoalSpeed(id uint32, raw uint16) error
	EnableTorque(id uint32) error
	DisableTorque(id uint32) error
	SetGoalPosition(id uint32, raw uint16) error
	Read(id uint32) (ServoReading, error)
}

type SeekConfig struct {
	Direction        int    // +1 or -1
	StepTicks        int    // goal increment per iteration
	MovingSpeed      uint16 // GOAL_SPEED cap (raw steps/s) so the joint creeps onto the stop instead of slamming it; 0 = servo max
	CurrentLimit     uint16 // current reading that signals pressing the stop
	FollowErrorTicks int    // goal-minus-actual lag that signals a blocked joint; robust when the torque cap holds current below CurrentLimit. 0 disables.
	ProgressTicks    int    // movement below this counts as "not advancing"
	PlateauReads     int    // consecutive blocked reads to declare a stop
	MaxTravelTicks   int    // abort if traveled this far without a stop
	SeamJumpTicks    int    // a position jump this large means the encoder seam is in the path
	StopAtCap        bool   // treat reaching MaxTravelTicks as a valid stop (a fenced, stopless joint) rather than a timeout
}

type SeekReason int

const (
	SeekOK SeekReason = iota
	SeekSeam
	SeekTimeout
	SeekReadFail
)

type SeekResult struct {
	RawStop uint16
	Reason  SeekReason
}

// SeekHardStop creeps GOAL_POSITION in Direction until the joint presses its
// hard stop, then returns that raw position. The joint is bound speed-wise by a
// GOAL_SPEED cap (MovingSpeed) so it eases onto the stop instead of slamming it.
// A stop is declared when, for PlateauReads consecutive reads, the joint is
// blocked by ANY of three signals: a current spike (CurrentLimit), the goal
// running ahead of the actual position (FollowErrorTicks — robust when the
// torque cap holds current below the spike threshold), or no position progress
// (ProgressTicks). It aborts on a near-4096 seam crossing, a read failure, or
// exceeding MaxTravel. The caller is responsible for having enabled torque
// (safe-goal-latched in core) and set the torque/overcurrent caps beforehand.
func SeekHardStop(c ServoClient, id uint32, cfg SeekConfig) SeekResult {
	if cfg.MovingSpeed > 0 {
		if err := c.SetGoalSpeed(id, cfg.MovingSpeed); err != nil {
			return SeekResult{Reason: SeekReadFail}
		}
	}
	start, err := c.Read(id)
	if err != nil || !start.OK {
		return SeekResult{Reason: SeekReadFail}
	}
	pos := start.PositionRaw
	goal := int(pos)
	last := int(pos)
	plateau := 0
	traveled := 0

	for {
		goal += cfg.Direction * cfg.StepTicks
		if goal < 0 {
			goal = 0
		}
		if goal > 4095 {
			goal = 4095
		}
		if err := c.SetGoalPosition(id, uint16(goal)); err != nil {
			return SeekResult{Reason: SeekReadFail}
		}
		r, err := c.Read(id)
		if err != nil || !r.OK {
			return SeekResult{Reason: SeekReadFail}
		}
		cur := int(r.PositionRaw)

		if absInt(cur-last) >= cfg.SeamJumpTicks {
			return SeekResult{Reason: SeekSeam}
		}
		pressing := r.CurrentRaw >= cfg.CurrentLimit
		lagging := cfg.FollowErrorTicks > 0 && absInt(goal-cur) >= cfg.FollowErrorTicks
		advancing := absInt(cur-last) >= cfg.ProgressTicks
		if pressing || lagging || !advancing {
			plateau++
			if plateau >= cfg.PlateauReads {
				return SeekResult{RawStop: r.PositionRaw, Reason: SeekOK}
			}
		} else {
			plateau = 0
		}

		traveled += absInt(cur - last)
		last = cur
		if traveled > cfg.MaxTravelTicks {
			if cfg.StopAtCap {
				return SeekResult{RawStop: r.PositionRaw, Reason: SeekOK}
			}
			return SeekResult{Reason: SeekTimeout}
		}
	}
}
