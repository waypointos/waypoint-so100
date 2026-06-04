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

// Step computes a clamped joint-angle delta for one dt given a body-frame
// twist. limits[i] = {lower, upper} soft limits (rad) for joints 0..3.
func (s *Solver) Step(q [5]float64, v Twist, limits [4][2]float64, dt float64) [4]float64 {
	// rotate the planar (x,y) command by the base-yaw offset
	c, sn := math.Cos(s.cfg.BaseYawOffsetRad), math.Sin(s.cfg.BaseYawOffsetRad)
	vx := c*v.Vx - sn*v.Vy
	vy := sn*v.Vx + c*v.Vy
	task := [4]float64{vx, vy, v.Vz, v.Wpitch}

	J := s.k.Jacobian(q)
	// Jt: task rows {vx=0, vy=1, vz=2, wy=4} x cols {0..3}
	rows := [4]int{0, 1, 2, 4}
	var Jt [4][4]float64
	for r := 0; r < 4; r++ {
		for col := 0; col < 4; col++ {
			Jt[r][col] = J[rows[r]][col]
		}
	}

	// A = JtT*Jt + lambda^2 I ; b = JtT*task ; solve A qdot = b
	lam2 := s.cfg.Damping * s.cfg.Damping
	var A [4][4]float64
	var b [4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			var sum float64
			for r := 0; r < 4; r++ {
				sum += Jt[r][i] * Jt[r][j]
			}
			if i == j {
				sum += lam2
			}
			A[i][j] = sum
		}
		var bs float64
		for r := 0; r < 4; r++ {
			bs += Jt[r][i] * task[r]
		}
		b[i] = bs
	}
	qdotFull := solve4(A, b)

	var out [4]float64
	for i := 0; i < 4; i++ {
		nq := q[i] + qdotFull[i]*dt
		if nq < limits[i][0] {
			nq = limits[i][0]
		} else if nq > limits[i][1] {
			nq = limits[i][1]
		}
		out[i] = nq - q[i]
	}
	return out
}

// solve4 solves a 4x4 linear system via Gaussian elimination with partial pivot.
func solve4(A [4][4]float64, b [4]float64) [4]float64 {
	for col := 0; col < 4; col++ {
		piv := col
		for r := col + 1; r < 4; r++ {
			if math.Abs(A[r][col]) > math.Abs(A[piv][col]) {
				piv = r
			}
		}
		A[col], A[piv] = A[piv], A[col]
		b[col], b[piv] = b[piv], b[col]
		if math.Abs(A[col][col]) < 1e-12 {
			continue
		}
		for r := col + 1; r < 4; r++ {
			f := A[r][col] / A[col][col]
			for c := col; c < 4; c++ {
				A[r][c] -= f * A[col][c]
			}
			b[r] -= f * b[col]
		}
	}
	var x [4]float64
	for r := 3; r >= 0; r-- {
		sum := b[r]
		for c := r + 1; c < 4; c++ {
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
