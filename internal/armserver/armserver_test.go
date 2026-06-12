package armserver

import (
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	waypointv1 "github.com/waypoint-rover/waypoint/protocol/gen/go/messages"

	"github.com/waypoint-rover/waypoint-so100/internal/calibration"
)

type fakeReader struct {
	raw map[uint32]uint16
	ok  map[uint32]bool
}

func (f *fakeReader) ReadRaw(id uint32) (uint16, bool) { return f.raw[id], f.ok[id] }

type fakeWriter struct {
	syncs    [][]*waypointv1.ServoGoal
	torqueOn map[uint32]bool
}

func (w *fakeWriter) SyncWriteGoals(goals []*waypointv1.ServoGoal) error {
	w.syncs = append(w.syncs, goals)
	return nil
}

func (w *fakeWriter) SetTorqueEnable(id uint32, on bool) error {
	if w.torqueOn == nil {
		w.torqueOn = map[uint32]bool{}
	}
	w.torqueOn[id] = on
	return nil
}

// cal builds an OK calibration whose zero is anchored so ThetaRad/RawFromRad
// round-trip cleanly for the test (zero raw maps to 0 rad).
func cal(id uint32) calibration.JointCal {
	return calibration.JointCal{ID: id, ZeroRaw: 2048, OK: true}
}

func TestStateMapsCalibratedJointAndFlagsUncalibrated(t *testing.T) {
	reader := &fakeReader{
		raw: map[uint32]uint16{1: 2048, 2: 3000},
		ok:  map[uint32]bool{1: true, 2: true},
	}
	cals := map[uint32]calibration.JointCal{1: cal(1)} // joint 2 uncalibrated
	s := New(reader, &fakeWriter{}, cals, map[uint32]string{1: "arm_1", 2: "arm_2"}, []uint32{1, 2})

	st := s.State()
	require.Len(t, st.GetJoints(), 2)

	assert.Equal(t, "arm_1", st.GetJoints()[0].GetName())
	assert.True(t, st.GetJoints()[0].GetCalibrated())
	assert.InDelta(t, 0.0, st.GetJoints()[0].GetPositionRad(), 1e-9) // raw 2048 == zero

	assert.Equal(t, "arm_2", st.GetJoints()[1].GetName())
	assert.False(t, st.GetJoints()[1].GetCalibrated())
	assert.Equal(t, 0.0, st.GetJoints()[1].GetPositionRad())
}

func TestCommandGoalsProduceOneSyncWriteWithTorque(t *testing.T) {
	w := &fakeWriter{}
	cals := map[uint32]calibration.JointCal{1: cal(1)}
	s := New(&fakeReader{}, w, cals, map[uint32]string{1: "arm_1"}, []uint32{1})

	cmd := &waypointv1.ArmCommand{Cmd: &waypointv1.ArmCommand_Goals{Goals: &waypointv1.ArmJointGoals{
		Goals: []*waypointv1.ArmJointGoal{{Name: "arm_1", PositionRad: 0.0}},
	}}}
	require.NoError(t, s.Command(cmd))

	require.Len(t, w.syncs, 1)
	require.Len(t, w.syncs[0], 1)
	assert.Equal(t, uint32(1), w.syncs[0][0].GetServoId())
	assert.Equal(t, uint32(2048), w.syncs[0][0].GetGoalPosition()) // 0 rad == zero raw
	assert.True(t, w.torqueOn[1])
}

func TestCommandRefusesUncalibratedJoint(t *testing.T) {
	w := &fakeWriter{}
	s := New(&fakeReader{}, w, map[uint32]calibration.JointCal{}, map[uint32]string{1: "arm_1"}, []uint32{1})

	cmd := &waypointv1.ArmCommand{Cmd: &waypointv1.ArmCommand_Goals{Goals: &waypointv1.ArmJointGoals{
		Goals: []*waypointv1.ArmJointGoal{{Name: "arm_1", PositionRad: 0.5}},
	}}}
	require.NoError(t, s.Command(cmd))
	assert.Empty(t, w.syncs, "uncalibrated joint must not produce a goal write")
}

func TestStopHaltsAndRelatchesPresentGoals(t *testing.T) {
	reader := &fakeReader{
		raw: map[uint32]uint16{1: 2200, 2: 1800},
		ok:  map[uint32]bool{1: true, 2: true},
	}
	w := &fakeWriter{}
	s := New(reader, w, map[uint32]calibration.JointCal{1: cal(1)}, map[uint32]string{1: "arm_1", 2: "arm_2"}, []uint32{1, 2})
	halted := false
	s.SetHalt(func() { halted = true })

	cmd := &waypointv1.ArmCommand{Cmd: &waypointv1.ArmCommand_Stop{Stop: true}}
	require.NoError(t, s.Command(cmd))

	assert.True(t, halted, "stop must freeze the teleop loop")
	require.Len(t, w.syncs, 1)
	require.Len(t, w.syncs[0], 2, "every readable joint re-latches, calibrated or not")
	assert.Equal(t, uint32(2200), w.syncs[0][0].GetGoalPosition())
	assert.Equal(t, uint32(1800), w.syncs[0][1].GetGoalPosition())
	assert.Empty(t, w.torqueOn, "stop must not change torque state")
}

func TestStopWithoutHaltHookStillRelatches(t *testing.T) {
	reader := &fakeReader{raw: map[uint32]uint16{1: 2100}, ok: map[uint32]bool{1: true}}
	w := &fakeWriter{}
	s := New(reader, w, map[uint32]calibration.JointCal{}, map[uint32]string{1: "arm_1"}, []uint32{1})

	cmd := &waypointv1.ArmCommand{Cmd: &waypointv1.ArmCommand_Stop{Stop: true}}
	require.NoError(t, s.Command(cmd))
	require.Len(t, w.syncs, 1)
}
