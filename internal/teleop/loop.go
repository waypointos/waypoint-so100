package teleop

import (
	"log/slog"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/waypointos/waypoint-so100/internal/calibration"
	"github.com/waypointos/waypoint-so100/internal/ik"
	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
)

// Sink emits a coordinated multi-joint goal write, per-joint moving-speed caps,
// and torque enable (all implemented by servo.Client). Torque enable is used by
// pose recall, which may run after the operator left torque off to pose by hand.
type Sink interface {
	SyncWriteGoals(goals []*so100v1.ServoGoal) error
	SetGoalSpeed(id uint32, raw uint16) error
	SetTorqueEnable(id uint32, on bool) error
}

type LoopConfig struct {
	StaleAfter     time.Duration
	MaxLinear      float64 // m/s cap on |planar+vertical|
	MaxPitch       float64 // rad/s cap
	MaxPan         float64 // rad/s, shoulder pan (joint 1) FPV direct view-sweep rate
	MaxRoll        float64 // rad/s, wrist roll (joint 5) hold-to-move rate
	MaxGrip        float64 // rad/s, gripper (joint 6) hold-to-move rate
	RampPerTick    float64 // max change in commanded twist per tick
	PanRampPerTick float64 // max change in the normalized pan rate per tick (FPV yaw smoothing); <=0 disables
	Dt             float64
	// RecallSpeed is the position-mode moving-speed cap (raw steps/s) used when a
	// saved pose is recalled, so the arm glides to the pose instead of darting
	// there at the servos' default max speed. ~800 steps/s ≈ 1.2 rad/s. Tune on
	// hardware; <=0 leaves the servos' prior cap.
	RecallSpeed uint16
	// ShareButton / OptionsButton are the gamepad buttons[] indices whose rising
	// edge recalls the "share" / "options" pose slots.
	ShareButton   int
	OptionsButton int
}

type Loop struct {
	cfg    LoopConfig
	sink   Sink
	cals   map[uint32]calibration.JointCal
	solver *ik.Solver
	k      ik.Kinematics

	mu          sync.Mutex
	last        *so100v1.GamepadSnapshot
	lastAt      time.Time
	q           [5]float64 // current joint estimate (rad); q[4] is wrist roll (joint 5)
	grip        float64    // current gripper estimate (rad), joint 6 (outside the IK chain)
	halted      bool
	prevTwist   ik.Twist                     // owned by the tick goroutine
	prevPan     float64                      // owned by the tick goroutine (FPV pan slew state)
	prevButtons []bool                       // last seen gamepad buttons, for recall rising-edge detection
	poses       map[string]map[uint32]uint16 // slot -> servo id -> raw, live pose set for recall

	// goalSpeedDirty is set when a pose recall left a reduced moving-speed cap on
	// the joints; the next teleop motion clears it back to max (0) so streamed
	// goals are not throttled (the acceleration register now governs smoothness).
	goalSpeedDirty bool

	recalling atomic.Bool // guards a single in-flight recall so a press can't overlap itself
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
	fired := l.poseEdgesLocked(s.GetButtons())
	l.mu.Unlock()
	// Recall outside the lock (it re-acquires l.mu) and off this goroutine: the
	// servo writes must not block the gamepad relay callback.
	for _, slot := range fired {
		go l.Recall(slot)
	}
}

// poseEdgesLocked returns the slots whose recall button just transitioned from
// released to pressed since the previous snapshot, and updates prevButtons. Any
// button going high is logged at Debug so the real Share/Options indices can be
// confirmed on hardware (the SDK's buttons[] layout is not the raw W3C one).
// Caller holds l.mu.
func (l *Loop) poseEdgesLocked(buttons []bool) []string {
	pressed := func(i int) bool { return i >= 0 && i < len(buttons) && buttons[i] }
	was := func(i int) bool { return i >= 0 && i < len(l.prevButtons) && l.prevButtons[i] }
	for i := range buttons {
		if buttons[i] && !was(i) {
			slog.Debug("teleop gamepad button down", "index", i)
		}
	}
	var fired []string
	if pressed(l.cfg.ShareButton) && !was(l.cfg.ShareButton) {
		fired = append(fired, "share")
	}
	if pressed(l.cfg.OptionsButton) && !was(l.cfg.OptionsButton) {
		fired = append(fired, "options")
	}
	l.prevButtons = append(l.prevButtons[:0], buttons...)
	return fired
}

// SetPoses swaps the live pose set the recall reads (slot -> servo id -> raw).
// main pushes a fresh snapshot whenever a pose is captured, deleted, or loaded.
func (l *Loop) SetPoses(poses map[string]map[uint32]uint16) {
	l.mu.Lock()
	l.poses = poses
	l.mu.Unlock()
}

// Recall drives joints 1..5 to the raw positions saved in slot, leaving the
// gripper (joint 6) where it is. Raw goals are written directly (bypassing IK
// and the rad<->raw round-trip) so replay is exact; the integrated IK estimate
// is then reseeded to the recalled pose so the next stick input does not snap
// back. A no-op if the slot is empty, the arm is uncalibrated, or a recall is
// already in flight. Invoked by the gamepad rising edge and the on-screen button.
func (l *Loop) Recall(slot string) {
	if !l.recalling.CompareAndSwap(false, true) {
		return // a recall is already running
	}
	defer l.recalling.Store(false)

	l.mu.Lock()
	pose := l.poses[slot]
	l.mu.Unlock()
	if len(pose) == 0 {
		return
	}
	// Failsafe: refuse to drive an uncalibrated arm, matching the tick path.
	if !l.calibrated() {
		slog.Warn("teleop pose recall refused: arm not calibrated", "slot", slot)
		return
	}

	goals := make([]*so100v1.ServoGoal, 0, 5)
	var nq [5]float64
	var seed [5]bool
	for _, id := range []uint32{1, 2, 3, 4, 5} { // joints 1..5; gripper (6) untouched
		raw, ok := pose[id]
		if !ok {
			continue
		}
		cal, hasCal := l.cals[id]
		if !hasCal || !cal.OK {
			continue
		}
		_ = l.sink.SetTorqueEnable(id, true) // operator may have left torque off after posing by hand
		if l.cfg.RecallSpeed > 0 {
			_ = l.sink.SetGoalSpeed(id, l.cfg.RecallSpeed)
		}
		goals = append(goals, &so100v1.ServoGoal{ServoId: id, GoalPosition: uint32(raw)})
		nq[id-1] = cal.ThetaRad(raw)
		seed[id-1] = true
	}
	if len(goals) == 0 {
		return
	}
	_ = l.sink.SyncWriteGoals(goals)

	// Reseed the IK reference to the recalled pose so a subsequent twist
	// integrates from where the arm now is, and drop ramp state. Mark the
	// moving-speed cap dirty so the next teleop motion clears the recall speed
	// back to max (see tick).
	l.mu.Lock()
	for i := 0; i < 5; i++ {
		if seed[i] {
			l.q[i] = nq[i]
		}
	}
	l.prevTwist = ik.Twist{}
	if l.cfg.RecallSpeed > 0 {
		l.goalSpeedDirty = true
	}
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
// integrates from the real pose. Ignored while teleop is actively commanding
// (see seedAllowedLocked): during motion the integrated q is the reference of
// record, and overwriting it with the lagging encoder read every cycle fights
// the integrator — the source of teleop jitter and sluggish creep.
func (l *Loop) SetJointEstimate(q [5]float64) {
	l.mu.Lock()
	if l.seedAllowedLocked() {
		l.q = q
	}
	l.mu.Unlock()
}

// SetGripEstimate seeds the gripper angle (joint 6, from jointstate) so the
// hold-to-move gripper integrates from the real position. Joint 6 is outside
// the IK chain, so it is tracked separately from q. Gated like SetJointEstimate.
func (l *Loop) SetGripEstimate(rad float64) {
	l.mu.Lock()
	if l.seedAllowedLocked() {
		l.grip = rad
	}
	l.mu.Unlock()
}

// Estimate returns the integrated joint estimate (rad; q[0..4] = joints 1..5,
// grip = joint 6) and whether teleop is actively commanding. While active, the
// joints publisher should render this instead of reading the servo bus, so the
// 50 Hz goal stream is not starved by position reads (the reads would be
// discarded by the reseed gate anyway).
func (l *Loop) Estimate() (q [5]float64, grip float64, active bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.q, l.grip, !l.seedAllowedLocked()
}

// seedAllowedLocked reports whether the sensor may overwrite the integrated
// reference. Only while teleop is idle — halted, no input yet, or input gone
// stale — so a fresh move always starts from the true measured pose, but an
// in-flight move integrates its own reference without sensor fight. Caller
// holds l.mu.
func (l *Loop) seedAllowedLocked() bool {
	if l.halted || l.last == nil {
		return true
	}
	return time.Since(l.lastAt) > l.cfg.StaleAfter
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
		l.prevPan = 0
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
	// Slew-limit the FPV pan rate too: it is a continuous analog axis (not a
	// digital hold-to-move like roll/gripper), so easing it in/out stops yaw
	// from lurching on a stick flick and filters residual stick noise — the
	// pan-specific half of the jitter. PanRampPerTick<=0 disables it.
	if l.cfg.PanRampPerTick > 0 {
		cmd.Pan = slew(l.prevPan, cmd.Pan, l.cfg.PanRampPerTick)
	}
	l.prevPan = cmd.Pan

	goals := make([]*so100v1.ServoGoal, 0, 6)

	// FPV pan: shoulder_pan (joint 1) is a direct hold-to-move rate that sweeps
	// the gripper camera. It is intentionally outside IK so "pan is just pan",
	// and the IK below reads the updated q[0] to resolve reach/pitch in the new
	// camera heading. Apply it before IK so the solver sees the new heading.
	if cmd.Pan != 0 {
		if nrad, goal, ok := l.directJoint(1, q[0], cmd.Pan*l.cfg.MaxPan*l.cfg.Dt); ok {
			q[0] = nrad
			goals = append(goals, goal)
		}
	}

	// Failsafe 3: IK joints 2..4 from the view-relative twist, soft-limit
	// clamped. Joint 1 (pan) is handled above and held fixed inside the solver.
	if cmd.Twist != (ik.Twist{}) {
		limits := l.softLimits()
		dq := l.solver.Step(q, cmd.Twist, limits, l.cfg.Dt)
		for i := 1; i < 4; i++ {
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
	dirty := l.goalSpeedDirty
	l.goalSpeedDirty = false
	l.mu.Unlock()

	// A prior pose recall left a reduced moving-speed cap on these joints. Clear
	// it back to max (0) on the first teleop motion so the streamed goals are not
	// throttled — smoothness now comes from the servos' acceleration register
	// (set once at startup), not a per-tick speed cap.
	if dirty {
		for _, g := range goals {
			_ = l.sink.SetGoalSpeed(g.GetServoId(), 0)
		}
	}

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

// slew moves prev toward want by at most step (a per-tick rate limiter).
func slew(prev, want, step float64) float64 {
	if want-prev > step {
		return prev + step
	}
	if prev-want > step {
		return prev - step
	}
	return want
}

func ramp(prev, want ik.Twist, step float64) ik.Twist {
	return ik.Twist{
		Vx:     slew(prev.Vx, want.Vx, step),
		Vy:     slew(prev.Vy, want.Vy, step),
		Vz:     slew(prev.Vz, want.Vz, step),
		Wpitch: slew(prev.Wpitch, want.Wpitch, step),
	}
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
