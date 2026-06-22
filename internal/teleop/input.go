package teleop

import (
	"math"

	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
	"github.com/waypointos/waypoint-so100/internal/ik"
)

type MapConfig struct {
	LinearScale   float64 // m/s per stick unit
	VerticalScale float64 // m/s per trigger unit
	PitchScale    float64 // rad/s per bumper
}

// Command is the per-frame interpretation: an IK twist (reach/lift/tilt in the
// camera view frame) plus direct-axis setpoints that bypass IK — shoulder pan
// (the FPV view sweep), wrist roll, and gripper.
type Command struct {
	Twist     ik.Twist
	Pan       float64 // -1..1, joint 1 shoulder_pan direct rate (sweeps the view)
	WristRoll float64 // rad/s, joint 5 direct
	Gripper   float64 // -1..1 close/open, joint 6 direct
}

func axis(a []float32, i int) float64 {
	if i < len(a) {
		return float64(a[i])
	}
	return 0
}
func btn(b []bool, i int) float64 {
	if i < len(b) && b[i] {
		return 1
	}
	return 0
}

const stickDeadzone = 0.08 // ignore analog-stick noise near center

// deadzone zeroes |v|<dz and rescales the remainder to a full 0..1 range, so
// motion eases in smoothly from rest instead of snapping on at the dz edge and
// so resting stick noise can't drive (and jitter) the joints.
func deadzone(v, dz float64) float64 {
	a := math.Abs(v)
	if a < dz {
		return 0
	}
	s := (a - dz) / (1 - dz)
	if v < 0 {
		return -s
	}
	return s
}

func MapInput(s *so100v1.GamepadSnapshot, cfg MapConfig) Command {
	rx := deadzone(axis(s.GetAxes(), 2), stickDeadzone)
	ry := deadzone(axis(s.GetAxes(), 3), stickDeadzone)
	lt, rt := axis(s.GetTriggers(), 0), axis(s.GetTriggers(), 1)
	// FPV: drive the gripper like an FPV drone — pan to aim the camera, then
	// reach along the view. Stick X yaws shoulder_pan (sweeps the view); stick Y
	// reaches in/out along the current heading (resolved view-relative in the IK
	// from the live pan angle). There is no lateral strafe — pan-then-reach
	// covers it, which is what feels right on a single forward-facing camera.
	// bumpers LB=4, RB=5 (tilt view) ; roll on buttons 2/3 ; gripper buttons 0/1.
	return Command{
		Twist: ik.Twist{
			Vx:     0,                            // no strafe in FPV; pan handles lateral
			Vy:     -ry * cfg.LinearScale,        // stick up = reach forward along the view
			Vz:     (rt - lt) * cfg.VerticalScale, // RT up, LT down (world vertical)
			Wpitch: (btn(s.GetButtons(), 5) - btn(s.GetButtons(), 4)) * cfg.PitchScale, // tilt the view
		},
		Pan:       rx, // stick right = pan the view right (scaled by MaxPan in the loop)
		WristRoll: btn(s.GetButtons(), 3) - btn(s.GetButtons(), 2),
		Gripper:   btn(s.GetButtons(), 1) - btn(s.GetButtons(), 0),
	}
}
