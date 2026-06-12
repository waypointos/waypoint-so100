// Package servobus adapts the waypoint SDK servo client to the so100-internal
// interfaces (calibration.ServoClient, teleop.Sink, jointstate.RawReader). The
// SDK speaks the same servo-control broker subjects; this only bridges widths
// (uint16 raw) and the so100 ServoGoal/ServoReading shapes.
package servobus

import (
	waypointv1 "github.com/waypoint-rover/waypoint/protocol/gen/go/messages"
	"github.com/waypoint-rover/waypoint/sdk/wpmodule"

	"github.com/waypoint-rover/waypoint-so100/internal/calibration"
	so100v1 "github.com/waypoint-rover/waypoint-so100/protocol/gen/go"
)

// Adapter wraps the SDK servo client behind the so100 module's interfaces.
type Adapter struct{ sv *wpmodule.ServoClient }

func New(sv *wpmodule.ServoClient) *Adapter { return &Adapter{sv: sv} }

func (a *Adapter) SetMode(id, mode uint32) error { return a.sv.SetMode(id, mode) }
func (a *Adapter) SetTorqueLimit(id uint32, raw uint16) error {
	return a.sv.SetTorqueLimit(id, uint32(raw))
}
func (a *Adapter) SetOvercurrentLimit(id uint32, raw uint16) error {
	return a.sv.SetOvercurrentLimit(id, uint32(raw))
}
func (a *Adapter) SetAngleLimits(id uint32, min, max uint16) error {
	return a.sv.SetAngleLimits(id, uint32(min), uint32(max))
}
func (a *Adapter) EnableTorque(id uint32) error  { return a.sv.SetTorqueEnable(id, true) }
func (a *Adapter) DisableTorque(id uint32) error { return a.sv.SetTorqueEnable(id, false) }
func (a *Adapter) SetGoalPosition(id uint32, raw uint16) error {
	return a.sv.SetGoalPosition(id, uint32(raw))
}

// SyncWriteGoals relays so100 goals to the SDK's waypoint.v1 sync write.
func (a *Adapter) SyncWriteGoals(goals []*so100v1.ServoGoal) error {
	out := make([]*waypointv1.ServoGoal, 0, len(goals))
	for _, g := range goals {
		out = append(out, &waypointv1.ServoGoal{ServoId: g.GetServoId(), GoalPosition: g.GetGoalPosition()})
	}
	return a.sv.SyncWriteGoals(out)
}

// ReadRaw satisfies jointstate.RawReader: present raw position and success.
func (a *Adapter) ReadRaw(id uint32) (uint16, bool) {
	r, err := a.Read(id)
	if err != nil || !r.OK {
		return 0, false
	}
	return r.PositionRaw, true
}

// Read requests one servo's raw state and maps it to the so100 reading shape.
// An absent (ok=false) servo state surfaces as OK=false, not sentinel zeros.
func (a *Adapter) Read(id uint32) (calibration.ServoReading, error) {
	st, err := a.sv.Read(id)
	if err != nil {
		return calibration.ServoReading{}, err
	}
	return calibration.ServoReading{
		PositionRaw: uint16(st.GetPositionRaw()),
		CurrentRaw:  uint16(st.GetCurrentRaw()),
		OK:          st.GetOk(),
	}, nil
}
