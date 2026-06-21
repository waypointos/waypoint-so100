package teleop

import (
	"testing"

	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
)

func TestMapInput_RightStickPansAndReaches(t *testing.T) {
	snap := &so100v1.GamepadSnapshot{
		Axes:     []float32{0, 0, 0.5, -0.5}, // right stick X=0.5 (right), Y=-0.5 (up)
		Triggers: []float32{0, 0},
		Buttons:  make([]bool, 8),
	}
	cmd := MapInput(snap, MapConfig{LinearScale: 0.1, VerticalScale: 0.1, PitchScale: 1.0})
	// FPV: stick X sweeps the view via shoulder pan (+Pan); stick Y reaches
	// forward along the view (+Vy). There is no lateral strafe — Vx stays 0.
	// Guards the stick-axis channel assignment against regressing.
	if cmd.Pan <= 0 {
		t.Fatalf("stick right should pan the view right (+Pan): %+v", cmd)
	}
	if cmd.Twist.Vy <= 0 {
		t.Fatalf("stick up should give +Vy (reach): %+v", cmd.Twist)
	}
	if cmd.Twist.Vx != 0 {
		t.Fatalf("FPV has no strafe; Vx must be 0: %+v", cmd.Twist)
	}
}

func TestMapInput_TriggersToVertical(t *testing.T) {
	snap := &so100v1.GamepadSnapshot{Axes: []float32{0, 0, 0, 0}, Triggers: []float32{0, 0.8}, Buttons: make([]bool, 8)}
	cmd := MapInput(snap, MapConfig{LinearScale: 0.1, VerticalScale: 0.1, PitchScale: 1.0})
	if cmd.Twist.Vz <= 0 {
		t.Fatalf("RT should raise the gripper: %+v", cmd.Twist)
	}
}
