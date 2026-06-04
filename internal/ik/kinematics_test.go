package ik

import (
	"math"
	"testing"
)

func TestFK_ZeroPoseReachIsPositive(t *testing.T) {
	k := SO100Kinematics()
	p := k.FK([5]float64{0, 0, 0, 0, 0}).Origin()
	// At the home pose the gripper must be at a nonzero reach from the base.
	if math.Hypot(p.X, p.Z) < 0.05 {
		t.Fatalf("unexpected home reach: %+v", p)
	}
}

func TestJacobian_MatchesFiniteDifference(t *testing.T) {
	k := SO100Kinematics()
	q := [5]float64{0.1, 0.3, -0.2, 0.15, 0.0}
	J := k.Jacobian(q) // 6x5
	const h = 1e-6
	base := k.FK(q).Origin()
	for j := 0; j < 4; j++ { // check the 4 IK joints' linear rows
		qd := q
		qd[j] += h
		pd := k.FK(qd).Origin()
		dvx := (pd.X - base.X) / h
		dvz := (pd.Z - base.Z) / h
		if math.Abs(J[0][j]-dvx) > 1e-3 || math.Abs(J[2][j]-dvz) > 1e-3 {
			t.Fatalf("jacobian col %d mismatch: J=(%v,%v) fd=(%v,%v)", j, J[0][j], J[2][j], dvx, dvz)
		}
	}
}
