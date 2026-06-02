package calibration

import "fmt"

// CalibrateConfig holds the powered-seek safety/tuning parameters. Defaults are
// conservative; tune on hardware.
type CalibrateConfig struct {
	TorqueLimit    uint16
	OvercurrentRaw uint16
	SpanTolerance  int
	Seek           SeekConfig
}

func DefaultCalibrateConfig() CalibrateConfig {
	return CalibrateConfig{
		TorqueLimit:    400,
		OvercurrentRaw: 600,
		SpanTolerance:  60,
		Seek: SeekConfig{
			StepTicks: 20, CurrentLimit: 500, ProgressTicks: 4,
			PlateauReads: 3, MaxTravelTicks: 4096, SeamJumpTicks: 1000,
		},
	}
}

// CalibrateJoint runs the full safe sequence for one joint and returns its
// derived calibration. Torque is always left OFF on return.
func CalibrateJoint(c ServoClient, spec JointSpec, cfg CalibrateConfig) (JointCal, error) {
	const modePosition = 0
	if err := c.SetMode(spec.ID, modePosition); err != nil {
		return JointCal{}, fmt.Errorf("set mode: %w", err)
	}
	if err := c.SetAngleLimits(spec.ID, 0, 4095); err != nil { // open full range for the seek
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

	up := cfg.Seek
	up.Direction = +1
	upRes := SeekHardStop(c, spec.ID, up)
	if upRes.Reason != SeekOK {
		return JointCal{ID: spec.ID}, seekErr("up", upRes.Reason)
	}
	down := cfg.Seek
	down.Direction = -1
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
