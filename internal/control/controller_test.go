package control

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/waypointos/waypoint-so100/internal/calibration"
)

// scriptedClient presses symmetric stops for every joint so a full run completes.
type scriptedClient struct{ lastGoal map[uint32]uint16 }

func newScripted() *scriptedClient { return &scriptedClient{lastGoal: map[uint32]uint16{}} }
func (s *scriptedClient) SetMode(uint32, uint32) error                { return nil }
func (s *scriptedClient) SetTorqueLimit(uint32, uint16) error         { return nil }
func (s *scriptedClient) SetOvercurrentLimit(uint32, uint16) error    { return nil }
func (s *scriptedClient) SetAngleLimits(uint32, uint16, uint16) error { return nil }
func (s *scriptedClient) EnableTorque(uint32) error                   { return nil }
func (s *scriptedClient) DisableTorque(uint32) error                  { return nil }
func (s *scriptedClient) SetGoalPosition(id uint32, raw uint16) error { s.lastGoal[id] = raw; return nil }
func (s *scriptedClient) Read(id uint32) (calibration.ServoReading, error) {
	g := s.lastGoal[id]
	switch {
	case g <= 800:
		return calibration.ServoReading{PositionRaw: 800, CurrentRaw: 900, OK: true}, nil
	case g >= 3200:
		return calibration.ServoReading{PositionRaw: 3200, CurrentRaw: 900, OK: true}, nil
	default:
		return calibration.ServoReading{PositionRaw: g, CurrentRaw: 50, OK: true}, nil
	}
}

func TestRunCalibration_CalibratesAllJointsAndPublishesStates(t *testing.T) {
	var states []calibration.JointCal
	ctrl := New(newScripted(), func(cals []calibration.JointCal, phase string, active uint32) {
		if phase == "done" {
			states = append([]calibration.JointCal{}, cals...)
		}
	})
	ctrl.RunCalibration()
	require.Len(t, states, len(calibration.SO100Joints))
}
