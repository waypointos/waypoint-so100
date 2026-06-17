package calibration

import "fmt"

// CalibrateConfig holds the powered-seek safety/tuning parameters. Defaults are
// conservative; tune on hardware.
type CalibrateConfig struct {
	TorqueLimit     uint16
	OvercurrentRaw  uint16
	SpanTolerance   int
	SeekMarginTicks int // travel allowed beyond a joint's URDF span before the seek aborts
	NoStopCapTicks  int // hard travel fence each way from center for stopless joints (wrist roll)
	Seek            SeekConfig
}

func DefaultCalibrateConfig() CalibrateConfig {
	return CalibrateConfig{
		TorqueLimit:     400,
		OvercurrentRaw:  600,
		SpanTolerance:   60,
		SeekMarginTicks: 200,  // ~18 deg of overshoot to reach and press the stop
		NoStopCapTicks:  1024, // 90 deg; wrist roll has no stop and must not snag the camera cable
		Seek: SeekConfig{
			StepTicks: 20, CurrentLimit: 500, ProgressTicks: 4,
			PlateauReads: 3, SeamJumpTicks: 1000,
			// MaxTravelTicks is set per joint in CalibrateJoint from its real range.
		},
	}
}

// CalibrateJoint runs the full safe sequence for one joint and returns its
// derived calibration. Torque is always left OFF on return. A jointed limb
// (HasHardStop) seeks both mechanical stops, bounded to its URDF range plus a
// small margin so a missed stop aborts at the joint's range instead of driving
// a full revolution into the floor. A stopless joint (wrist roll) is fenced to
// a window around its centered start so the seek cannot snag the camera cable.
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

	// Jointed limbs open to the full encoder range; a stopless joint is fenced in
	// the servo itself to +/- NoStopCapTicks of center, a hardware backstop that
	// holds even if the seek logic is wrong.
	loLimit, hiLimit := uint16(0), uint16(4095)
	if !spec.HasHardStop {
		loLimit = clampTick(center - cfg.NoStopCapTicks)
		hiLimit = clampTick(center + cfg.NoStopCapTicks)
	}
	if err := c.SetAngleLimits(spec.ID, loLimit, hiLimit); err != nil {
		return JointCal{}, fmt.Errorf("angle limits: %w", err)
	}
	if err := c.SetTorqueLimit(spec.ID, cfg.TorqueLimit); err != nil {
		return JointCal{}, fmt.Errorf("torque limit: %w", err)
	}
	if err := c.SetOvercurrentLimit(spec.ID, cfg.OvercurrentRaw); err != nil {
		return JointCal{}, fmt.Errorf("overcurrent: %w", err)
	}
	if err := c.EnableTorque(spec.ID); err != nil { // safe-goal-latched in core
		return JointCal{}, fmt.Errorf("enable torque: %w", err)
	}
	defer c.DisableTorque(spec.ID) //nolint:errcheck

	if spec.HasHardStop {
		return seekBothStops(c, spec, cfg)
	}
	return seekFencedWindow(c, spec, cfg, center)
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

// seekFencedWindow probes a stopless joint within +/- NoStopCapTicks of its
// centered start. Reaching the fence is a valid limit (StopAtCap); a real stop
// found earlier is used instead. The descent budget is sized so the total
// excursion stays within the fence regardless of where the ascent stopped.
func seekFencedWindow(c ServoClient, spec JointSpec, cfg CalibrateConfig, center int) (JointCal, error) {
	up := cfg.Seek
	up.Direction = +1
	up.MaxTravelTicks = cfg.NoStopCapTicks
	up.StopAtCap = true
	upRes := SeekHardStop(c, spec.ID, up)
	if upRes.Reason != SeekOK {
		return JointCal{ID: spec.ID}, seekErr("up", upRes.Reason)
	}

	down := cfg.Seek
	down.Direction = -1
	down.MaxTravelTicks = (int(upRes.RawStop) - center) + cfg.NoStopCapTicks
	down.StopAtCap = true
	downRes := SeekHardStop(c, spec.ID, down)
	if downRes.Reason != SeekOK {
		return JointCal{ID: spec.ID}, seekErr("down", downRes.Reason)
	}

	rawMin, rawMax := downRes.RawStop, upRes.RawStop
	if rawMin > rawMax {
		rawMin, rawMax = rawMax, rawMin
	}
	return DeriveCentered(spec, uint16(center), rawMin, rawMax), nil
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
