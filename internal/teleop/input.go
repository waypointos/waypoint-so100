package teleop

import (
	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
	"github.com/waypointos/waypoint-so100/internal/ik"
)

type MapConfig struct {
	LinearScale   float64 // m/s per stick unit
	VerticalScale float64 // m/s per trigger unit
	PitchScale    float64 // rad/s per bumper
}

// Command is the per-frame interpretation: an IK twist plus direct-axis
// setpoints that bypass IK (wrist roll, gripper).
type Command struct {
	Twist     ik.Twist
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

func MapInput(s *so100v1.GamepadSnapshot, cfg MapConfig) Command {
	rx, ry := axis(s.GetAxes(), 2), axis(s.GetAxes(), 3)
	lt, rt := axis(s.GetTriggers(), 0), axis(s.GetTriggers(), 1)
	// bumpers LB=4, RB=5 ; roll on buttons 2/3 ; gripper buttons 0/1
	return Command{
		Twist: ik.Twist{
			Vx:     -ry * cfg.LinearScale, // stick up = forward
			Vy:     rx * cfg.LinearScale,
			Vz:     (rt - lt) * cfg.VerticalScale, // RT up, LT down
			Wpitch: (btn(s.GetButtons(), 5) - btn(s.GetButtons(), 4)) * cfg.PitchScale,
		},
		WristRoll: btn(s.GetButtons(), 3) - btn(s.GetButtons(), 2),
		Gripper:   btn(s.GetButtons(), 1) - btn(s.GetButtons(), 0),
	}
}
