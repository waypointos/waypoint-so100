package ik

import (
	"math"
	"testing"
)

func almost(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

func TestRotZThenTranslate(t *testing.T) {
	m := RotZ(math.Pi / 2).Mul(Translate(1, 0, 0))
	p := m.Apply(Vec3{0, 0, 0})
	// origin of the translated frame, rotated 90 deg about z -> (0,1,0)
	if !almost(p.X, 0) || !almost(p.Y, 1) || !almost(p.Z, 0) {
		t.Fatalf("got %+v", p)
	}
}
