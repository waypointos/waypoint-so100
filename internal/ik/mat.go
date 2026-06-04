package ik

import "math"

type Vec3 struct{ X, Y, Z float64 }

// Mat4 is a 4x4 homogeneous transform stored row-major.
type Mat4 [16]float64

func Identity() Mat4 { return Mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1} }

func Translate(x, y, z float64) Mat4 {
	m := Identity()
	m[3], m[7], m[11] = x, y, z
	return m
}

func RotX(a float64) Mat4 {
	c, s := math.Cos(a), math.Sin(a)
	return Mat4{1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1}
}
func RotY(a float64) Mat4 {
	c, s := math.Cos(a), math.Sin(a)
	return Mat4{c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1}
}
func RotZ(a float64) Mat4 {
	c, s := math.Cos(a), math.Sin(a)
	return Mat4{c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
}

func (a Mat4) Mul(b Mat4) Mat4 {
	var o Mat4
	for r := 0; r < 4; r++ {
		for c := 0; c < 4; c++ {
			var s float64
			for k := 0; k < 4; k++ {
				s += a[r*4+k] * b[k*4+c]
			}
			o[r*4+c] = s
		}
	}
	return o
}

// Apply transforms a point (w=1).
func (m Mat4) Apply(p Vec3) Vec3 {
	return Vec3{
		m[0]*p.X + m[1]*p.Y + m[2]*p.Z + m[3],
		m[4]*p.X + m[5]*p.Y + m[6]*p.Z + m[7],
		m[8]*p.X + m[9]*p.Y + m[10]*p.Z + m[11],
	}
}

// Origin returns the translation column.
func (m Mat4) Origin() Vec3 { return Vec3{m[3], m[7], m[11]} }

// AxisZ returns the rotated unit z-axis (column 2 of the rotation block).
func (m Mat4) AxisZ() Vec3 { return Vec3{m[2], m[6], m[10]} }

func Cross(a, b Vec3) Vec3 {
	return Vec3{a.Y*b.Z - a.Z*b.Y, a.Z*b.X - a.X*b.Z, a.X*b.Y - a.Y*b.X}
}
func Sub(a, b Vec3) Vec3 { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }
