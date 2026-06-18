package calibration

import "fmt"

// CalibrateConfig holds the powered-seek safety/tuning parameters. Defaults are
// conservative; tune on hardware.
type CalibrateConfig struct {
	TorqueLimit     uint16
	JointTorque     map[uint32]uint16 // per-joint torque override; falls back to TorqueLimit
	OvercurrentRaw  uint16
	SpanTolerance   int
	SeekMarginTicks int // travel allowed beyond a joint's URDF span before the seek aborts
	NoStopCapTicks  int // soft-limit half-window each way from center for stopless joints (wrist roll)
	Seek            SeekConfig
}

// TorqueFor returns the seek torque cap for a joint, honoring a per-joint
// override. The shoulder joints are capped lower so that, if the seek ever drives
// the arm into the chassis or the ground, the servo cannot generate enough force
// to lift the rover (which masks the stop) before the seek detects the block.
func (c CalibrateConfig) TorqueFor(id uint32) uint16 {
	if t, ok := c.JointTorque[id]; ok {
		return t
	}
	return c.TorqueLimit
}

func DefaultCalibrateConfig() CalibrateConfig {
	return CalibrateConfig{
		TorqueLimit: 350,
		// Shoulder pan/lift bear the whole arm; keep their seek torque low so a
		// misaimed seek binds and is detected instead of lifting the rover.
		JointTorque:     map[uint32]uint16{1: 250, 2: 250},
		OvercurrentRaw:  600,
		SpanTolerance:   60,
		SeekMarginTicks: 200, // ~18 deg of overshoot to reach and press the stop
		// 512 ticks = 45 deg each way = a 90 deg total window. Wrist roll has no
		// hard stop; it is fenced here and, crucially, is NEVER power-seeked, so
		// the camera cable is never wound past this window.
		NoStopCapTicks: 512,
		Seek: SeekConfig{
			StepTicks: 20, MovingSpeed: 200, CurrentLimit: 350,
			FollowErrorTicks: 80, ProgressTicks: 4,
			PlateauReads: 3, SeamJumpTicks: 1000,
			// MaxTravelTicks is set per joint in CalibrateJoint from its real range.
		},
	}
}

// CalibrateJoint runs the safe sequence for one joint and returns its derived
// calibration. Torque is always left OFF on return.
//
// A jointed limb (HasHardStop) seeks both mechanical stops, bounded to its URDF
// range plus a small margin so a missed stop aborts at the joint's range instead
// of driving a full revolution into the floor.
//
// A stopless joint (wrist roll) has no stop to find, so seeking it would only
// wind the gripper-camera cable for nothing. It is NOT powered or moved: we set
// the servo's hardware angle-limit fence and derive soft limits straight from its
// resting center, leaving the cable untouched.
func CalibrateJoint(c ServoClient, spec JointSpec, cfg CalibrateConfig) (JointCal, error) {
	const modePosition = 0
	if err := c.SetMode(spec.ID, modePosition); err != nil {
		return JointCal{}, fmt.Errorf("set mode: %w", err)
	}

	start, err := c.Read(spec.ID)
	if err != nil || !start.OK {
		return JointCal{ID: spec.ID}, fmt.Errorf("read start position")
	}
	center := int(start.PositionRaw)

	if !spec.HasHardStop {
		return centerStoplessJoint(c, spec, cfg, center)
	}

	if err := c.SetAngleLimits(spec.ID, 0, 4095); err != nil { // open the full range for the seek
		return JointCal{}, fmt.Errorf("angle limits: %w", err)
	}
	if err := c.SetTorqueLimit(spec.ID, cfg.TorqueFor(spec.ID)); err != nil {
		return JointCal{}, fmt.Errorf("torque limit: %w", err)
	}
	if err := c.SetOvercurrentLimit(spec.ID, cfg.OvercurrentRaw); err != nil {
		return JointCal{}, fmt.Errorf("overcurrent: %w", err)
	}
	if err := c.EnableTorque(spec.ID); err != nil { // safe-goal-latched in core
		return JointCal{}, fmt.Errorf("enable torque: %w", err)
	}
	defer c.DisableTorque(spec.ID) //nolint:errcheck

	return seekBothStops(c, spec, cfg)
}

// centerStoplessJoint calibrates a joint with no hard stop (wrist roll) without
// moving it. The resting position is the zero; the soft limits are a fixed
// +/- NoStopCapTicks window around it, and the same window is written to the
// servo's hardware angle limits as a backstop. No torque is enabled, so the
// gripper-camera cable is never wound during calibration.
func centerStoplessJoint(c ServoClient, spec JointSpec, cfg CalibrateConfig, center int) (JointCal, error) {
	lo := clampTick(center - cfg.NoStopCapTicks)
	hi := clampTick(center + cfg.NoStopCapTicks)
	if err := c.SetAngleLimits(spec.ID, lo, hi); err != nil {
		return JointCal{}, fmt.Errorf("angle limits: %w", err)
	}
	return DeriveCentered(spec, uint16(center), lo, hi), nil
}

// seekBothStops ramps to the upper then lower mechanical stop, bounding each
// seek to the joint's URDF span plus a margin.
func seekBothStops(c ServoClient, spec JointSpec, cfg CalibrateConfig) (JointCal, error) {
	travelCap := ExpectedSpanTicks(spec) + cfg.SeekMarginTicks

	up := cfg.Seek
	up.Direction = +1
	up.MaxTravelTicks = travelCap
	upRes := SeekHardStop(c, spec.ID, up)
	if upRes.Reason != SeekOK {
		return JointCal{ID: spec.ID}, seekErr("up", upRes.Reason)
	}
	down := cfg.Seek
	down.Direction = -1
	down.MaxTravelTicks = travelCap
	downRes := SeekHardStop(c, spec.ID, down)
	if downRes.Reason != SeekOK {
		return JointCal{ID: spec.ID}, seekErr("down", downRes.Reason)
	}

	rawMin, rawMax := downRes.RawStop, upRes.RawStop
	if rawMin > rawMax {
		rawMin, rawMax = rawMax, rawMin
	}
	return Derive(spec, rawMin, rawMax, cfg.SpanTolerance), nil
}

func clampTick(t int) uint16 {
	if t < 0 {
		return 0
	}
	if t > 4095 {
		return 4095
	}
	return uint16(t)
}

func seekErr(dir string, r SeekReason) error {
	switch r {
	case SeekSeam:
		return fmt.Errorf("%s: seam in workspace", dir)
	case SeekTimeout:
		return fmt.Errorf("%s: timeout (no hard stop)", dir)
	default:
		return fmt.Errorf("%s: read failure", dir)
	}
}
