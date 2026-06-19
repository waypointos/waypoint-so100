package teleop

import (
	"math"
	"sync"
	"time"

	"github.com/waypointos/waypoint-so100/internal/calibration"
	"github.com/waypointos/waypoint-so100/internal/ik"
	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
)

// Sink emits a coordinated multi-joint goal write (implemented by servo.Client).
type Sink interface {
	SyncWriteGoals(goals []*so100v1.ServoGoal) error
}

type LoopConfig struct {
	StaleAfter  time.Duration
	MaxLinear   float64 // m/s cap on |planar+vertical|
	MaxPitch    float64 // rad/s cap
	MaxRoll     float64 // rad/s, wrist roll (joint 5) hold-to-move rate
	MaxGrip     float64 // rad/s, gripper (joint 6) hold-to-move rate
	RampPerTick float64 // max change in commanded twist per tick
	Dt          float64
}

type Loop struct {
	cfg    LoopConfig
	sink   Sink
	cals   map[uint32]calibration.JointCal
	solver *ik.Solver
	k      ik.Kinematics

	mu        sync.Mutex
	last      *so100v1.GamepadSnapshot
	lastAt    time.Time
	q         [5]float64 // current joint estimate (rad); q[4] is wrist roll (joint 5)
	grip      float64    // current gripper estimate (rad), joint 6 (outside the IK chain)
	halted    bool
	prevTwist ik.Twist // owned by the tick goroutine
}

func NewLoop(cfg LoopConfig, sink Sink, cals map[uint32]calibration.JointCal, k ik.Kinematics) *Loop {
	return &Loop{
		cfg: cfg, sink: sink, cals: cals, k: k,
		solver: ik.NewSolver(k, ik.SolverConfig{Damping: 0.06, BaseYawOffsetRad: -math.Pi / 4}),
	}
}

func (l *Loop) SetInput(s *so100v1.GamepadSnapshot, at time.Time) {
	l.mu.Lock()
	l.last, l.lastAt = s, at
	l.halted = false // fresh operator input resumes a halted loop
	l.mu.Unlock()
}

// Halt drops the current input and suppresses ticks until fresh input
// arrives, so an ArmCommand stop freezes motion immediately; servos hold
// their last written goal.
func (l *Loop) Halt() {
	l.mu.Lock()
	l.last = nil
	l.halted = true
	l.mu.Unlock()
}

// SetJointEstimate seeds the current joint angles (from jointstate) so IK
// integrates from the real pose.
func (l *Loop) SetJointEstimate(q [5]float64) {
	l.mu.Lock()
	l.q = q
	l.mu.Unlock()
}

// SetGripEstimate seeds the gripper angle (joint 6, from jointstate) so the
// hold-to-move gripper integrates from the real position. Joint 6 is outside
// the IK chain, so it is tracked separately from q.
func (l *Loop) SetGripEstimate(rad float64) {
	l.mu.Lock()
	l.grip = rad
	l.mu.Unlock()
}

func (l *Loop) calibrated() bool {
	for _, id := range []uint32{1, 2, 3, 4} {
		if _, ok := l.cals[id]; !ok {
			return false
		}
	}
	return true
}

func (l *Loop) tick(now time.Time) {
	l.mu.Lock()
	snap, at, q, grip, halted := l.last, l.lastAt, l.q, l.grip, l.halted
	l.mu.Unlock()

	if halted {
		l.prevTwist = ik.Twist{} // discard ramp state so resume starts from rest
		return
	}

	// Failsafe 5: refuse to move uncalibrated (IK joints 1..4 gate all motion).
	if !l.calibrated() {
		return
	}

	var cmd Command
	// Failsafe 1: staleness.
	if snap != nil && now.Sub(at) <= l.cfg.StaleAfter {
		cmd = MapInput(snap, MapConfig{LinearScale: l.cfg.MaxLinear, VerticalScale: l.cfg.MaxLinear, PitchScale: l.cfg.MaxPitch})
	} // else cmd stays zero

	// Failsafe 2: magnitude clamp + ramp (twist only).
	cmd.Twist = clampTwist(cmd.Twist, l.cfg.MaxLinear, l.cfg.MaxPitch)
	cmd.Twist = ramp(l.prevTwist, cmd.Twist, l.cfg.RampPerTick)
	l.prevTwist = cmd.Twist

	goals := make([]*so100v1.ServoGoal, 0, 6)

	// Failsafe 3: IK joints 1..4 from the Cartesian twist, soft-limit clamped.
	if cmd.Twist != (ik.Twist{}) {
		limits := l.softLimits()
		dq := l.solver.Step(q, cmd.Twist, limits, l.cfg.Dt)
		for i := 0; i < 4; i++ {
			q[i] += dq[i]
			id := uint32(i + 1)
			raw := l.cals[id].RawFromRad(q[i])
			goals = append(goals, &so100v1.ServoGoal{ServoId: id, GoalPosition: uint32(raw)})
		}
	}

	// Joint 5 wrist roll: direct hold-to-move rate, clamped to its soft limits.
	if cmd.WristRoll != 0 {
		if nrad, goal, ok := l.directJoint(5, q[4], cmd.WristRoll*l.cfg.MaxRoll*l.cfg.Dt); ok {
			q[4] = nrad
			goals = append(goals, goal)
		}
	}

	// Joint 6 gripper: direct hold-to-move rate, clamped to its soft limits.
	if cmd.Gripper != 0 {
		if nrad, goal, ok := l.directJoint(6, grip, cmd.Gripper*l.cfg.MaxGrip*l.cfg.Dt); ok {
			grip = nrad
			goals = append(goals, goal)
		}
	}

	if len(goals) == 0 {
		return // nothing to do; servos hold last goal
	}

	l.mu.Lock()
	l.q = q
	l.grip = grip
	l.mu.Unlock()

	_ = l.sink.SyncWriteGoals(goals)
}

// directJoint integrates a non-IK joint (wrist roll, gripper) by dRad from cur,
// clamps to its calibrated soft limits, and returns the new angle plus the goal
// write. ok is false when the joint has no usable calibration, in which case it
// is skipped (per-joint guard) while the IK joints still move.
func (l *Loop) directJoint(id uint32, cur, dRad float64) (nrad float64, goal *so100v1.ServoGoal, ok bool) {
	cal, has := l.cals[id]
	if !has || !cal.OK {
		return cur, nil, false
	}
	lim := cal.SoftLimitsRad()
	lo, hi := lim[0], lim[1]
	if lo > hi { // wrist roll's sign convention can invert lower/upper
		lo, hi = hi, lo
	}
	n := cur + dRad
	if n < lo {
		n = lo
	} else if n > hi {
		n = hi
	}
	return n, &so100v1.ServoGoal{ServoId: id, GoalPosition: uint32(cal.RawFromRad(n))}, true
}

func (l *Loop) softLimits() [4][2]float64 {
	var out [4][2]float64
	for i := 0; i < 4; i++ {
		out[i] = l.cals[uint32(i+1)].SoftLimitsRad()
	}
	return out
}

func clampTwist(v ik.Twist, maxLin, maxPitch float64) ik.Twist {
	mag := math.Sqrt(v.Vx*v.Vx + v.Vy*v.Vy + v.Vz*v.Vz)
	if mag > maxLin && mag > 0 {
		s := maxLin / mag
		v.Vx, v.Vy, v.Vz = v.Vx*s, v.Vy*s, v.Vz*s
	}
	if v.Wpitch > maxPitch {
		v.Wpitch = maxPitch
	} else if v.Wpitch < -maxPitch {
		v.Wpitch = -maxPitch
	}
	return v
}

func ramp(prev, want ik.Twist, step float64) ik.Twist {
	lim := func(p, w float64) float64 {
		if w-p > step {
			return p + step
		}
		if p-w > step {
			return p - step
		}
		return w
	}
	return ik.Twist{Vx: lim(prev.Vx, want.Vx), Vy: lim(prev.Vy, want.Vy), Vz: lim(prev.Vz, want.Vz), Wpitch: lim(prev.Wpitch, want.Wpitch)}
}

// Run drives tick at the configured rate until stop is closed; wired in main.go.
func (l *Loop) Run(stop <-chan struct{}) {
	ticker := time.NewTicker(time.Duration(l.cfg.Dt * float64(time.Second)))
	defer ticker.Stop()
	for {
		select {
		case <-stop:
			return
		case now := <-ticker.C:
			l.tick(now)
		}
	}
}
