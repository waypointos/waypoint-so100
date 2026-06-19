package teleop

import (
	"testing"
	"time"

	"github.com/waypointos/waypoint-so100/internal/calibration"
	"github.com/waypointos/waypoint-so100/internal/ik"
	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
)

type recSink struct {
	goals  [][]*so100v1.ServoGoal
	speeds map[uint32]uint16 // last moving-speed cap written per servo
}

func (r *recSink) SyncWriteGoals(g []*so100v1.ServoGoal) error {
	r.goals = append(r.goals, g)
	return nil
}

func (r *recSink) SetGoalSpeed(id uint32, raw uint16) error {
	if r.speeds == nil {
		r.speeds = map[uint32]uint16{}
	}
	r.speeds[id] = raw
	return nil
}

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
		StaleAfter: 150 * time.Millisecond,
		MaxLinear:  0.15, MaxPitch: 1.0, MaxRoll: 1.0, MaxGrip: 1.5, RampPerTick: 0.02,
		Dt: 0.02, GoalSpeedHeadroom: 1.3,
	}
}

// goalFor returns the goal for servo id within a single sync-write batch, or nil.
func goalFor(batch []*so100v1.ServoGoal, id uint32) *so100v1.ServoGoal {
	for _, g := range batch {
		if g.GetServoId() == id {
			return g
		}
	}
	return nil
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

// WristRoll = buttons[3]-buttons[2]; Gripper = buttons[1]-buttons[0].
func TestLoop_WristRollEmitsJoint5Goal(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Buttons: []bool{false, false, false, true}}, now) // roll open, zero twist
	l.tick(now)
	if len(s.goals) != 1 {
		t.Fatalf("expected one sync write, got %d", len(s.goals))
	}
	if goalFor(s.goals[0], 5) == nil {
		t.Fatal("wrist-roll input must write a joint-5 goal")
	}
}

func TestLoop_GripperEmitsJoint6Goal(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Buttons: []bool{false, true}}, now) // gripper open, zero twist
	l.tick(now)
	if len(s.goals) != 1 {
		t.Fatalf("expected one sync write, got %d", len(s.goals))
	}
	if goalFor(s.goals[0], 6) == nil {
		t.Fatal("gripper input must write a joint-6 goal")
	}
}

// Regression for the restructure: roll/gripper must write even with zero twist.
func TestLoop_RollAndGripperWithoutTwist(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Buttons: []bool{false, true, false, true}}, now) // grip + roll, no axes/triggers
	l.tick(now)
	if len(s.goals) != 1 {
		t.Fatalf("expected one sync write, got %d", len(s.goals))
	}
	batch := s.goals[0]
	if goalFor(batch, 5) == nil || goalFor(batch, 6) == nil {
		t.Fatal("expected both joint-5 and joint-6 goals")
	}
	if goalFor(batch, 1) != nil {
		t.Fatal("zero twist must not write IK joints")
	}
}

// Holding roll one direction must saturate at the soft limit, never past it.
func TestLoop_WristRollClampedToSoftLimit(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	cal := fixedCalibration()[5]
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Buttons: []bool{false, false, false, true}}, now)
	for i := 0; i < 1000; i++ {
		l.tick(now) // same now keeps input fresh
	}
	if len(s.goals) == 0 {
		t.Fatal("expected writes while holding roll")
	}
	for _, batch := range s.goals {
		g := goalFor(batch, 5)
		if g == nil {
			continue
		}
		if pos := uint16(g.GetGoalPosition()); pos < cal.SoftMin || pos > cal.SoftMax {
			t.Fatalf("joint-5 goal %d outside soft limits [%d,%d]", pos, cal.SoftMin, cal.SoftMax)
		}
	}
	last := goalFor(s.goals[len(s.goals)-1], 5)
	if last == nil || uint16(last.GetGoalPosition()) != cal.SoftMax {
		t.Fatalf("roll should saturate at soft max %d, got %v", cal.SoftMax, last)
	}
}

// Each moved joint gets a finite moving-speed cap written before its goal, so
// the servo glides rather than darting at its default max speed. A zero cap
// (the register's "max speed") would be the jitter this guards against.
func TestLoop_WritesFiniteGoalSpeed(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 1, -1}, Triggers: []float32{0, 0}}, now)
	// Ramp a few ticks so the twist (and thus the per-joint deltas) is non-trivial.
	for i := 0; i < 5; i++ {
		l.tick(now)
	}
	if len(s.speeds) == 0 {
		t.Fatal("expected per-joint moving-speed caps to be written")
	}
	for id, raw := range s.speeds {
		if raw == 0 {
			t.Fatalf("joint %d speed cap is 0 (= servo max speed); must be finite", id)
		}
	}
}

// With headroom disabled the loop must not touch the speed registers, leaving
// the servos at whatever cap they already hold (back-compat / opt-out).
func TestLoop_NoGoalSpeedWhenHeadroomZero(t *testing.T) {
	s := &recSink{}
	cfg := testLoopCfg()
	cfg.GoalSpeedHeadroom = 0
	l := NewLoop(cfg, s, fixedCalibration(), SO100KinematicsForTest())
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 1, -1}, Triggers: []float32{0, 0}}, now)
	l.tick(now)
	if len(s.goals) == 0 {
		t.Fatal("expected motion")
	}
	if len(s.speeds) != 0 {
		t.Fatalf("headroom 0 must write no speed caps; got %d", len(s.speeds))
	}
}

// While actively teleoperating, the lagging sensor must NOT overwrite the
// integrated reference (jitter/creep guard).
func TestLoop_SeedIgnoredWhileActive(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s)
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 0.5, 0}}, now) // fresh -> active
	l.SetJointEstimate([5]float64{1, 1, 1, 1, 1})
	l.SetGripEstimate(1)
	if l.q[0] != 0 || l.grip != 0 {
		t.Fatalf("active teleop must ignore sensor reseed; got q=%v grip=%v", l.q, l.grip)
	}
}

// When idle (no input, halted, or stale), the sensor seeds the reference so a
// fresh move starts from the true pose.
func TestLoop_SeedAppliedWhenIdle(t *testing.T) {
	s := &recSink{}
	l := newTestLoop(s) // no input yet -> idle
	l.SetJointEstimate([5]float64{1, 1, 1, 1, 1})
	l.SetGripEstimate(2)
	if l.q[0] != 1 || l.grip != 2 {
		t.Fatalf("idle loop must accept sensor reseed; got q=%v grip=%v", l.q, l.grip)
	}
	// Stale input is also idle.
	l.SetInput(&so100v1.GamepadSnapshot{Axes: []float32{0, 0, 0.5, 0}}, time.Now().Add(-time.Second))
	l.SetJointEstimate([5]float64{3, 3, 3, 3, 3})
	if l.q[0] != 3 {
		t.Fatalf("stale input is idle; reseed must apply; got q=%v", l.q)
	}
}

// An uncalibrated gripper is skipped while the IK joints still move.
func TestLoop_UncalibratedGripperSkipsJoint6(t *testing.T) {
	s := &recSink{}
	cals := fixedCalibration()
	delete(cals, 6)
	l := NewLoop(testLoopCfg(), s, cals, SO100KinematicsForTest())
	now := time.Now()
	l.SetInput(&so100v1.GamepadSnapshot{
		Axes:    []float32{0, 0, 0.5, -0.5}, // some twist -> IK joints move
		Buttons: []bool{false, true},        // gripper open -> should be dropped
	}, now)
	l.tick(now)
	if len(s.goals) != 1 {
		t.Fatalf("expected one sync write, got %d", len(s.goals))
	}
	if goalFor(s.goals[0], 1) == nil {
		t.Fatal("IK joints should still move")
	}
	if goalFor(s.goals[0], 6) != nil {
		t.Fatal("uncalibrated gripper must be skipped")
	}
}
