package ik

import (
	"math"
	"testing"
)

func TestResolvedRate_ForwardTwistMovesEE(t *testing.T) {
	k := SO100Kinematics()
	s := NewSolver(k, SolverConfig{Damping: 0.05, BaseYawOffsetRad: -math.Pi / 4})
	q := [5]float64{0.0, 0.2, -0.3, 0.1, 0}
	before := k.FK(q).Origin()
	// command a forward reach (view-relative +Vy) for one step
	qdot := s.Step(q, Twist{Vy: 0.05}, [4][2]float64{{-2, 2}, {-2, 2}, {-2, 2}, {-2, 2}}, 0.02)
	for i := 0; i < 4; i++ {
		q[i] += qdot[i]
	}
	after := k.FK(q).Origin()
	if math.Hypot(after.X-before.X, after.Z-before.Z) < 1e-4 {
		t.Fatal("EE did not move for a forward twist")
	}
}

func TestResolvedRate_ClampsToSoftLimits(t *testing.T) {
	k := SO100Kinematics()
	s := NewSolver(k, SolverConfig{Damping: 0.05})
	q := [5]float64{0, 0, 0, 0, 0}
	// IK joints 2..4 (indices 1..3) sit at their upper limit and must not be
	// pushed past it. Joint 1 (index 0) is pan: never driven by IK, so its delta
	// must be exactly zero.
	limits := [4][2]float64{{-2, 2}, {-0.01, 0.0}, {-0.01, 0.0}, {-0.01, 0.0}}
	qdot := s.Step(q, Twist{Vy: 1.0, Vz: 1.0, Wpitch: 1.0}, limits, 0.02)
	if qdot[0] != 0 {
		t.Fatalf("joint 1 (pan) must not be driven by IK; got dq0=%v", qdot[0])
	}
	for i := 1; i < 4; i++ {
		if q[i]+qdot[i] > 0.0+1e-9 {
			t.Fatalf("joint %d exceeded its upper soft limit: %v", i, q[i]+qdot[i])
		}
	}
}

func TestResolvedRate_NearSingularStaysBounded(t *testing.T) {
	k := SO100Kinematics()
	s := NewSolver(k, SolverConfig{Damping: 0.08})
	q := [5]float64{0, 0, 0, 0, 0} // a stretched/singular-ish pose
	qdot := s.Step(q, Twist{Vx: 1.0, Vy: 1.0, Vz: 1.0, Wpitch: 1.0}, [4][2]float64{{-3, 3}, {-3, 3}, {-3, 3}, {-3, 3}}, 0.02)
	for i := 0; i < 4; i++ {
		if math.Abs(qdot[i]) > 0.2 { // DLS keeps the step bounded
			t.Fatalf("unbounded qdot[%d]=%v near singularity", i, qdot[i])
		}
	}
}
