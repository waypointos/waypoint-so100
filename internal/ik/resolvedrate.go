package ik

import "math"

type Twist struct{ Vx, Vy, Vz, Wpitch float64 }

type SolverConfig struct {
	Damping          float64 // DLS lambda
	BaseYawOffsetRad float64 // Rz applied to the commanded twist (id:1 mount; -pi/4)
}

type Solver struct {
	k   Kinematics
	cfg SolverConfig
}

func NewSolver(k Kinematics, cfg SolverConfig) *Solver { return &Solver{k: k, cfg: cfg} }

// Step computes a clamped joint-angle delta for one dt given a camera/view-frame
// twist. limits[i] = {lower, upper} soft limits (rad) for joints 0..3.
//
// FPV control. The gripper camera is bolted to the end of the arm, so its
// horizontal heading is set by shoulder pan (joint 1). Pan is driven directly
// by the operator outside IK ("pan is just pan") and held fixed here; this
// solver resolves the remaining joints 2..4 in the camera's current heading:
//   - the planar reach (Vx,Vy) is rotated by the live heading = mount offset +
//     current pan q[0], so "forward" tracks where the camera looks regardless
//     of the rover's orientation;
//   - the wrist-pitch command (Wpitch, "tilt the view") is applied about the
//     camera's lateral axis, which also rotates with pan.
//
// At pan=0 this reduces to the pre-FPV mapping (reach rotated by the mount
// offset, pitch about base +y), so the home-pose behavior is unchanged.
func (s *Solver) Step(q [5]float64, v Twist, limits [4][2]float64, dt float64) [4]float64 {
	pan := q[0]

	// Planar reach in the camera heading (mount offset + live pan). If reach
	// runs opposite to where the camera looks on hardware, flip the sign of pan
	// in `yaw` (pan-axis sense vs. the Rz convention).
	yaw := s.cfg.BaseYawOffsetRad + pan
	c, sn := math.Cos(yaw), math.Sin(yaw)
	vx := c*v.Vx - sn*v.Vy
	vy := sn*v.Vx + c*v.Vy

	// Wrist pitch about the camera's lateral axis. At pan=0 the axis is base +y
	// (wy) as before; rotating it by pan keeps "tilt up" tilting the view up at
	// any pan angle.
	cp, sp := math.Cos(pan), math.Sin(pan)
	wx := -sp * v.Wpitch
	wy := cp * v.Wpitch

	J := s.k.Jacobian(q)
	// Task rows {vx,vy,vz,wx,wy}; solve joints {2,3,4} (cols 1..3). Joint 1
	// (pan) is excluded — it is commanded directly by the teleop loop, so the
	// IK never fights it. Reach(1) + vertical(1) + pitch(1) == 3 joints.
	rows := [5]int{0, 1, 2, 3, 4}
	task := [5]float64{vx, vy, v.Vz, wx, wy}
	cols := [3]int{1, 2, 3}

	var Jt [5][3]float64
	for r := 0; r < 5; r++ {
		for ci, col := range cols {
			Jt[r][ci] = J[rows[r]][col]
		}
	}

	// A = JtT*Jt + lambda^2 I ; b = JtT*task ; solve A qdot = b (damped least sq).
	lam2 := s.cfg.Damping * s.cfg.Damping
	var A [3][3]float64
	var b [3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			var sum float64
			for r := 0; r < 5; r++ {
				sum += Jt[r][i] * Jt[r][j]
			}
			if i == j {
				sum += lam2
			}
			A[i][j] = sum
		}
		var bs float64
		for r := 0; r < 5; r++ {
			bs += Jt[r][i] * task[r]
		}
		b[i] = bs
	}
	qdot := solve3(A, b)

	// out[0] stays 0: joint 1 (pan) is not driven by IK.
	var out [4]float64
	for ci, col := range cols {
		nq := q[col] + qdot[ci]*dt
		if nq < limits[col][0] {
			nq = limits[col][0]
		} else if nq > limits[col][1] {
			nq = limits[col][1]
		}
		out[col] = nq - q[col]
	}
	return out
}

// solve3 solves a 3x3 linear system via Gaussian elimination with partial pivot.
func solve3(A [3][3]float64, b [3]float64) [3]float64 {
	for col := 0; col < 3; col++ {
		piv := col
		for r := col + 1; r < 3; r++ {
			if math.Abs(A[r][col]) > math.Abs(A[piv][col]) {
				piv = r
			}
		}
		A[col], A[piv] = A[piv], A[col]
		b[col], b[piv] = b[piv], b[col]
		if math.Abs(A[col][col]) < 1e-12 {
			continue
		}
		for r := col + 1; r < 3; r++ {
			f := A[r][col] / A[col][col]
			for c := col; c < 3; c++ {
				A[r][c] -= f * A[col][c]
			}
			b[r] -= f * b[col]
		}
	}
	var x [3]float64
	for r := 2; r >= 0; r-- {
		sum := b[r]
		for c := r + 1; c < 3; c++ {
			sum -= A[r][c] * x[c]
		}
		if math.Abs(A[r][r]) < 1e-12 {
			x[r] = 0
		} else {
			x[r] = sum / A[r][r]
		}
	}
	return x
}
