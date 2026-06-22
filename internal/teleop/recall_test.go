package teleop

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func recallLoop(s Sink) *Loop {
	cfg := testLoopCfg()
	cfg.RecallSpeed = 800
	cfg.ShareButton, cfg.OptionsButton = 8, 9
	return NewLoop(cfg, s, fixedCalibration(), SO100KinematicsForTest())
}

func TestRecall_WritesExactRawForJoints1to5LeavesGripper(t *testing.T) {
	s := &recSink{}
	l := recallLoop(s)
	pose := map[uint32]uint16{1: 1500, 2: 1600, 3: 1700, 4: 1800, 5: 1900, 6: 1200}
	l.SetPoses(map[string]map[uint32]uint16{"share": pose})

	l.Recall("share")

	require.Len(t, s.goals, 1)
	batch := s.goals[0]
	for _, id := range []uint32{1, 2, 3, 4, 5} {
		g := goalFor(batch, id)
		require.NotNilf(t, g, "joint %d goal missing", id)
		require.Equal(t, uint32(pose[id]), g.GetGoalPosition(), "joint %d raw must be exact", id)
		require.Truef(t, s.torque[id], "joint %d torque should be enabled", id)
		require.Equal(t, uint16(800), s.speeds[id], "joint %d speed cap", id)
	}
	require.Nil(t, goalFor(batch, 6), "gripper (joint 6) must be left alone")
	require.False(t, s.torque[6], "gripper torque must be untouched")
}

func TestRecall_ReseedsIKEstimate(t *testing.T) {
	s := &recSink{}
	l := recallLoop(s)
	pose := map[uint32]uint16{1: 1500, 2: 1600, 3: 1700, 4: 1800, 5: 1900}
	l.SetPoses(map[string]map[uint32]uint16{"share": pose})

	l.Recall("share")

	cals := fixedCalibration()
	for _, id := range []uint32{1, 2, 3, 4, 5} {
		require.InDelta(t, cals[id].ThetaRad(pose[id]), l.q[id-1], 1e-9, "q[%d] reseeded", id-1)
	}
}

func TestRecall_UnknownOrEmptySlotIsNoop(t *testing.T) {
	s := &recSink{}
	l := recallLoop(s)
	l.SetPoses(map[string]map[uint32]uint16{"share": {1: 1500}})
	l.Recall("options") // not assigned
	require.Empty(t, s.goals)
}

func TestRecall_RefusedWhenUncalibrated(t *testing.T) {
	s := &recSink{}
	cfg := testLoopCfg()
	cfg.RecallSpeed = 800
	l := NewLoop(cfg, s, missingJointCalibration(), SO100KinematicsForTest())
	l.SetPoses(map[string]map[uint32]uint16{"share": {1: 1500, 2: 1600}})
	l.Recall("share")
	require.Empty(t, s.goals, "uncalibrated arm must not move")
}

func TestPoseEdges_RisingEdgeOncePerPress(t *testing.T) {
	l := recallLoop(&recSink{})
	buttons := make([]bool, 10)

	require.Empty(t, l.poseEdgesLocked(buttons), "nothing pressed")

	buttons[8] = true
	require.Equal(t, []string{"share"}, l.poseEdgesLocked(buttons), "share rising edge")
	require.Empty(t, l.poseEdgesLocked(buttons), "held button must not re-fire")

	buttons[8] = false
	require.Empty(t, l.poseEdgesLocked(buttons), "release is not an edge")

	buttons[9] = true
	require.Equal(t, []string{"options"}, l.poseEdgesLocked(buttons), "options rising edge")
}
