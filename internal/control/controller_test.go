package control

import (
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/waypointos/waypoint-so100/internal/calibration"
)

// sweepClient mimics an operator backdriving every joint between two extremes
// while torque is off. Read alternates between a low and a high tick (per joint)
// and counts reads so a test can wait until the sweep has been sampled.
type sweepClient struct {
	reads     atomic.Int64
	torqueOff atomic.Int64
	fences    atomic.Int64

	mu   sync.Mutex
	high map[uint32]bool
}

func newSweep() *sweepClient { return &sweepClient{high: map[uint32]bool{}} }

func (s *sweepClient) SetMode(uint32, uint32) error             { return nil }
func (s *sweepClient) SetTorqueLimit(uint32, uint16) error      { return nil }
func (s *sweepClient) SetOvercurrentLimit(uint32, uint16) error { return nil }
func (s *sweepClient) SetGoalSpeed(uint32, uint16) error        { return nil }
func (s *sweepClient) EnableTorque(uint32) error                { return nil }
func (s *sweepClient) DisableTorque(uint32) error               { s.torqueOff.Add(1); return nil }
func (s *sweepClient) SetGoalPosition(uint32, uint16) error     { return nil }

func (s *sweepClient) SetAngleLimits(uint32, uint16, uint16) error { s.fences.Add(1); return nil }

func (s *sweepClient) Read(id uint32) (calibration.ServoReading, error) {
	s.reads.Add(1)
	s.mu.Lock()
	high := s.high[id]
	s.high[id] = !high
	s.mu.Unlock()
	pos := uint16(1000)
	if high {
		pos = 3000
	}
	return calibration.ServoReading{PositionRaw: pos, CurrentRaw: 50, OK: true}, nil
}

func TestRecording_DisablesTorqueDerivesAndReports(t *testing.T) {
	client := newSweep()
	var (
		mu     sync.Mutex
		done   []calibration.JointCal
		phases []string
	)
	var homeSeen bool
	ctrl := New(client, func(cals []calibration.JointCal, phase string, hs bool) {
		mu.Lock()
		defer mu.Unlock()
		phases = append(phases, phase)
		if hs {
			homeSeen = true
		}
		if phase == "done" {
			done = append([]calibration.JointCal{}, cals...)
		}
	})
	ctrl.sample = 200 * time.Microsecond
	ctrl.pubEvery = 500 * time.Microsecond

	ctrl.StartRecording()
	require.True(t, ctrl.Recording(), "bus must read as in-use while recording")
	// Torque must be cut on every joint so the operator can move the arm.
	require.GreaterOrEqual(t, client.torqueOff.Load(), int64(len(calibration.SO100Joints)))

	// Capture a home pose, then keep sweeping to capture both extremes per joint.
	require.Eventually(t, func() bool { return client.reads.Load() > 50 }, time.Second, time.Millisecond)
	ctrl.SetHome()
	require.Eventually(t, func() bool {
		mu.Lock()
		defer mu.Unlock()
		return homeSeen
	}, time.Second, time.Millisecond, "home_set must propagate while recording")
	require.Eventually(t, func() bool { return client.reads.Load() > 400 },
		time.Second, time.Millisecond)

	ctrl.FinishRecording()
	require.Eventually(t, func() bool {
		mu.Lock()
		defer mu.Unlock()
		return done != nil
	}, time.Second, time.Millisecond)

	require.Eventually(t, func() bool { return !ctrl.Recording() }, time.Second, time.Millisecond,
		"bus must be released after the session ends")

	mu.Lock()
	defer mu.Unlock()
	require.Len(t, done, len(calibration.SO100Joints))
	require.Contains(t, phases, "recording")
	// All joints read, homed, and swept a full 1000..3000 span -> all clean.
	for _, c := range done {
		require.True(t, c.OK, "joint %d should be OK, got %q", c.ID, c.FlagReason)
	}
	// Clean joints get their soft limits fenced into the servos.
	require.GreaterOrEqual(t, client.fences.Load(), int64(len(calibration.SO100Joints)))
}

func TestFinishWithoutSession_IsNoop(t *testing.T) {
	ctrl := New(newSweep(), func([]calibration.JointCal, string, bool) {})
	ctrl.FinishRecording() // must not panic or block
	ctrl.Abort()
	ctrl.SetHome()
}
