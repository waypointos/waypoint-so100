package teleop

import (
	"testing"
	"time"

	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
	"github.com/waypointos/waypoint-so100/internal/calibration"
	"github.com/waypointos/waypoint-so100/internal/ik"
)

type recSink struct{ goals [][]*so100v1.ServoGoal }

func (r *recSink) SyncWriteGoals(g []*so100v1.ServoGoal) error { r.goals = append(r.goals, g); return nil }

func SO100KinematicsForTest() ik.Kinematics { return ik.SO100Kinematics() }

func fixedCalibration() map[uint32]calibration.JointCal {
	out := map[uint32]calibration.JointCal{}
	for _, id := range []uint32{1, 2, 3, 4, 5, 6} {
		spec := calibration.SO100Joints[id-1]
		out[id] = calibration.DeriveHome(spec, 2048, 100, 3996)
	}
	return out
}

func missingJointCalibration() map[uint32]calibration.JointCal {
	cals := fixedCalibration()
	delete(cals, 3) // an arm joint is uncalibrated
	return cals
}

func testLoopCfg() LoopConfig {
	return LoopConfig{
		StaleAfter:  150 * time.Millisecond,
		MaxLinear:   0.15, MaxPitch: 1.0, RampPerTick: 0.02,
		Dt: 0.02,
	}
}

func newTestLoop(sink Sink) *Loop {
	return NewLoop(testLoopCfg(), sink, fixedCalibration(), SO100KinematicsForTest())
}

func TestLoop_StaleInputProducesNoMotion(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 1, 0}}, time.Now().Add(-time.Second)) // stale
	l.tick(time.Now())
	if len(s.goals) != 0 {
		t.Fatalf("stale input must not move the arm; got %d writes", len(s.goals))
	}
}

func TestLoop_FreshInputEmitsOneSyncWrite(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 0.5, -0.5}, Triggers: []float32{0, 0}}, now)
	l.tick(now)
	if len(s.goals) != 1 {
		t.Fatalf("expected exactly one sync write, got %d", len(s.goals))
	}
}

func TestLoop_RefusesWhenJointUncalibrated(t *testing.T) {
	s := &recSink{}
	l := NewLoop(testLoopCfg(), s, missingJointCalibration(), SO100KinematicsForTest())
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 0.5, 0}}, now)
	l.tick(now)
	if len(s.goals) != 0 {
		t.Fatal("must refuse to move with an uncalibrated joint")
	}
}
