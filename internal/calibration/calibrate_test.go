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
func (p *twoStopPlant) SetGoalSpeed(uint32, uint16) error           { return nil }
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

// stoplessPlant records whether the stopless joint was ever powered or moved. A
// correct calibration of a no-hard-stop joint touches neither: it only sets the
// angle-limit fence and reads the resting position.
type stoplessPlant struct {
	pos              uint16
	loLimit, hiLimit uint16
	torqued, moved   bool
}

func (p *stoplessPlant) SetMode(uint32, uint32) error             { return nil }
func (p *stoplessPlant) SetTorqueLimit(uint32, uint16) error      { return nil }
func (p *stoplessPlant) SetOvercurrentLimit(uint32, uint16) error { return nil }
func (p *stoplessPlant) SetGoalSpeed(uint32, uint16) error        { return nil }
func (p *stoplessPlant) SetAngleLimits(_ uint32, min, max uint16) error {
	p.loLimit, p.hiLimit = min, max
	return nil
}
func (p *stoplessPlant) EnableTorque(uint32) error             { p.torqued = true; return nil }
func (p *stoplessPlant) DisableTorque(uint32) error            { return nil }
func (p *stoplessPlant) SetGoalPosition(uint32, uint16) error  { p.moved = true; return nil }
func (p *stoplessPlant) Read(uint32) (ServoReading, error) {
	return ServoReading{PositionRaw: p.pos, CurrentRaw: 50, OK: true}, nil
}

func TestCalibrateJoint_StoplessJointIsCenteredWithoutMoving(t *testing.T) {
	spec := JointSpec{ID: 5, Name: "wrist_roll", LowerRad: -2.74385, UpperRad: 2.84121, HasHardStop: false}
	p := &stoplessPlant{pos: 2048}
	cfg := DefaultCalibrateConfig()
	cal, err := CalibrateJoint(p, spec, cfg)
	require.NoError(t, err)
	require.True(t, cal.OK)

	// The cable-snagging failure mode: the joint must never be powered or driven.
	require.False(t, p.torqued, "stopless joint must not enable torque")
	require.False(t, p.moved, "stopless joint must not be commanded to move")

	// The resting position is the zero, and the fence is exactly +/- the window.
	require.InDelta(t, 0, cal.ThetaRad(2048), 1e-9)
	require.Equal(t, uint16(2048-cfg.NoStopCapTicks), p.loLimit)
	require.Equal(t, uint16(2048+cfg.NoStopCapTicks), p.hiLimit)
	require.Equal(t, uint16(2048-cfg.NoStopCapTicks), cal.RawMin)
	require.Equal(t, uint16(2048+cfg.NoStopCapTicks), cal.RawMax)
}
