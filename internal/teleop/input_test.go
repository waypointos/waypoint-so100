package teleop

import (
	"testing"

	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
)

func TestMapInput_RightStickToPlanarXY(t *testing.T) {
	snap := &so100v1.GamepadSnapshot{
		Axes:     []float32{0, 0, 0.5, -0.5}, // right stick X=0.5 (right), Y=-0.5 (up)
		Triggers: []float32{0, 0},
		Buttons:  make([]bool, 8),
	}
	cmd := MapInput(snap, MapConfig{LinearScale: 0.1, VerticalScale: 0.1, PitchScale: 1.0})
	// Horizontal (X right) drives lateral/pan (+Vx); vertical (Y up) drives
	// reach (+Vy). Guards the stick-axis channel assignment against regressing
	// back to the swapped wiring.
	if cmd.Twist.Vx <= 0 {
		t.Fatalf("stick right should give +Vx (pan/lateral): %+v", cmd.Twist)
	}
	if cmd.Twist.Vy <= 0 {
		t.Fatalf("stick up should give +Vy (reach): %+v", cmd.Twist)
	}
}

func TestMapInput_TriggersToVertical(t *testing.T) {
	snap := &so100v1.GamepadSnapshot{Axes: []float32{0, 0, 0, 0}, Triggers: []float32{0, 0.8}, Buttons: make([]bool, 8)}
	cmd := MapInput(snap, MapConfig{LinearScale: 0.1, VerticalScale: 0.1, PitchScale: 1.0})
	if cmd.Twist.Vz <= 0 {
		t.Fatalf("RT should raise the gripper: %+v", cmd.Twist)
	}
}
