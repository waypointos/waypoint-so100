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
	// command +x in rover frame for one step
	qdot := s.Step(q, Twist{Vx: 0.05}, [4][2]float64{{-2, 2}, {-2, 2}, {-2, 2}, {-2, 2}}, 0.02)
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
	// a joint already at its upper limit must not be pushed past it
	limits := [4][2]float64{{-0.01, 0.0}, {-2, 2}, {-2, 2}, {-2, 2}}
	qdot := s.Step(q, Twist{Vx: 1.0, Vz: 1.0}, limits, 0.02)
	if q[0]+qdot[0] > 0.0+1e-9 {
		t.Fatalf("joint 0 exceeded its upper soft limit: %v", q[0]+qdot[0])
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
