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
	EnableTorque(id uint32) error
	DisableTorque(id uint32) error
	SetGoalPosition(id uint32, raw uint16) error
	Read(id uint32) (ServoReading, error)
}

type SeekConfig struct {
	Direction      int    // +1 or -1
	StepTicks      int    // goal increment per iteration
	CurrentLimit   uint16 // current reading that signals pressing the stop
	ProgressTicks  int    // movement below this counts as "not advancing"
	PlateauReads   int    // consecutive non-advancing reads to declare a stop
	MaxTravelTicks int    // abort if traveled this far without a stop
	SeamJumpTicks  int    // a position jump this large means the encoder seam is in the path
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

// SeekHardStop ramps GOAL_POSITION in Direction until the joint presses its hard
// stop (current spike or position plateau), then returns that raw position. It
// aborts on a near-4096 seam crossing, a read failure, or exceeding MaxTravel.
// The caller is responsible for having enabled torque (safe-goal-latched in core)
// and set the torque/overcurrent caps beforehand.
func SeekHardStop(c ServoClient, id uint32, cfg SeekConfig) SeekResult {
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
		advancing := absInt(cur-last) >= cfg.ProgressTicks
		if pressing || !advancing {
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
			return SeekResult{Reason: SeekTimeout}
		}
	}
}
