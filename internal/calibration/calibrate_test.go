package calibration

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// twoStopPlant presses a lower stop when commanded down and an upper stop up.
type twoStopPlant struct {
	pos       uint16
	low, high uint16
	lastGoal  uint16
}

func (p *twoStopPlant) SetMode(uint32, uint32) error                { return nil }
func (p *twoStopPlant) SetTorqueLimit(uint32, uint16) error         { return nil }
func (p *twoStopPlant) SetOvercurrentLimit(uint32, uint16) error    { return nil }
func (p *twoStopPlant) SetAngleLimits(uint32, uint16, uint16) error { return nil }
func (p *twoStopPlant) EnableTorque(uint32) error                   { return nil }
func (p *twoStopPlant) DisableTorque(uint32) error                  { return nil }
func (p *twoStopPlant) SetGoalPosition(_ uint32, raw uint16) error  { p.lastGoal = raw; return nil }
func (p *twoStopPlant) Read(uint32) (ServoReading, error) {
	switch {
	case p.lastGoal <= p.low:
		return ServoReading{PositionRaw: p.low, CurrentRaw: 900, OK: true}, nil
	case p.lastGoal >= p.high:
		return ServoReading{PositionRaw: p.high, CurrentRaw: 900, OK: true}, nil
	default:
		return ServoReading{PositionRaw: p.lastGoal, CurrentRaw: 50, OK: true}, nil
	}
}

func TestCalibrateJoint_DerivesFromBothStops(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533, HasHardStop: true}
	p := &twoStopPlant{pos: 2000, low: 820, high: 3170}
	cfg := DefaultCalibrateConfig()
	cal, err := CalibrateJoint(p, spec, cfg)
	require.NoError(t, err)
	require.InDelta(t, 820, int(cal.RawMin), 60)
	require.InDelta(t, 3170, int(cal.RawMax), 60)
	require.InDelta(t, spec.LowerRad, cal.ThetaRad(cal.RawMin), 2e-2)
}

// freePlant spins freely: position tracks the goal and current never spikes, so
// the only thing that can end a seek is the travel fence.
type freePlant struct {
	loLimit, hiLimit uint16
	pos              uint16
}

func (p *freePlant) SetMode(uint32, uint32) error             { return nil }
func (p *freePlant) SetTorqueLimit(uint32, uint16) error      { return nil }
func (p *freePlant) SetOvercurrentLimit(uint32, uint16) error { return nil }
func (p *freePlant) SetAngleLimits(_ uint32, min, max uint16) error {
	p.loLimit, p.hiLimit = min, max
	return nil
}
func (p *freePlant) EnableTorque(uint32) error  { return nil }
func (p *freePlant) DisableTorque(uint32) error { return nil }
func (p *freePlant) SetGoalPosition(_ uint32, raw uint16) error {
	if raw < p.loLimit { // servo clamps goals to its angle limits
		raw = p.loLimit
	}
	if raw > p.hiLimit {
		raw = p.hiLimit
	}
	p.pos = raw
	return nil
}
func (p *freePlant) Read(uint32) (ServoReading, error) {
	return ServoReading{PositionRaw: p.pos, CurrentRaw: 50, OK: true}, nil
}

func TestCalibrateJoint_FencesStoplessJoint(t *testing.T) {
	spec := JointSpec{ID: 5, Name: "wrist_roll", LowerRad: -2.74385, UpperRad: 2.84121, HasHardStop: false}
	p := &freePlant{loLimit: 0, hiLimit: 4095, pos: 2048}
	cfg := DefaultCalibrateConfig()
	cal, err := CalibrateJoint(p, spec, cfg)
	require.NoError(t, err)
	require.True(t, cal.OK)

	// Center is the zero, and the measured window never escapes the +/-90 deg fence.
	require.InDelta(t, 0, cal.ThetaRad(2048), 1e-9)
	require.GreaterOrEqual(t, int(cal.RawMin), 2048-cfg.NoStopCapTicks-cfg.Seek.StepTicks)
	require.LessOrEqual(t, int(cal.RawMax), 2048+cfg.NoStopCapTicks+cfg.Seek.StepTicks)
	// The servo's own angle limits were fenced to the window too.
	require.Equal(t, uint16(2048-cfg.NoStopCapTicks), p.loLimit)
	require.Equal(t, uint16(2048+cfg.NoStopCapTicks), p.hiLimit)
}
