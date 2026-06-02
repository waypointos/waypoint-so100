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
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	p := &twoStopPlant{pos: 2000, low: 820, high: 3170}
	cfg := DefaultCalibrateConfig()
	cal, err := CalibrateJoint(p, spec, cfg)
	require.NoError(t, err)
	require.InDelta(t, 820, int(cal.RawMin), 60)
	require.InDelta(t, 3170, int(cal.RawMax), 60)
	require.InDelta(t, spec.LowerRad, cal.ThetaRad(cal.RawMin), 2e-2)
}
