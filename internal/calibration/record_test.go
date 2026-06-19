package calibration

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// scriptedReader replays a fixed position sequence per joint, one entry per Read
// call (SetHome and each Sample consume one), holding the last value once
// exhausted. Convention in these tests: entry 0 is the home pose, the rest are
// the sweep.
type scriptedReader struct {
	seq map[uint32][]uint16
	i   map[uint32]int
}

func newScriptedReader(seq map[uint32][]uint16) *scriptedReader {
	return &scriptedReader{seq: seq, i: map[uint32]int{}}
}

func (s *scriptedReader) SetMode(uint32, uint32) error                { return nil }
func (s *scriptedReader) SetTorqueLimit(uint32, uint16) error         { return nil }
func (s *scriptedReader) SetOvercurrentLimit(uint32, uint16) error    { return nil }
func (s *scriptedReader) SetAngleLimits(uint32, uint16, uint16) error { return nil }
func (s *scriptedReader) SetGoalSpeed(uint32, uint16) error           { return nil }
func (s *scriptedReader) EnableTorque(uint32) error                   { return nil }
func (s *scriptedReader) DisableTorque(uint32) error                  { return nil }
func (s *scriptedReader) SetGoalPosition(uint32, uint16) error        { return nil }
func (s *scriptedReader) Read(id uint32) (ServoReading, error) {
	vals, ok := s.seq[id]
	if !ok || len(vals) == 0 {
		return ServoReading{}, nil // not OK
	}
	idx := s.i[id]
	if idx >= len(vals) {
		idx = len(vals) - 1
	}
	s.i[id]++
	return ServoReading{PositionRaw: vals[idx], CurrentRaw: 50, OK: true}, nil
}

func driveAll(rec *Recorder, r *scriptedReader, n int) {
	for i := 0; i < n; i++ {
		rec.Sample(r)
	}
}

func TestRecorder_DerivesFromHomeAndSweep(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533, HasHardStop: true}
	upper := uint16(820 + ExpectedSpanTicks(spec))
	// entry 0 (2000) = home; the rest sweep up to `upper` and down to 820.
	r := newScriptedReader(map[uint32][]uint16{2: {2000, 2500, upper, 1500, 820}})
	rec := NewRecorder([]JointSpec{spec})
	rec.SetHome(r)
	driveAll(rec, r, 4)

	cal := rec.Results(DefaultRecordConfig())[0]
	require.True(t, cal.OK)
	require.InDelta(t, 0, cal.ThetaRad(2000), 1e-9) // home reads as 0 rad
	require.Equal(t, uint16(820), cal.RawMin)
	require.Equal(t, upper, cal.RawMax)
}

func TestRecorder_UnwrapsEncoderSeam(t *testing.T) {
	// A joint whose travel crosses the 0/4095 seam: 3900 -> wraps -> 600. The two
	// extremes sit on opposite sides of the seam, so no single linear raw->rad map
	// over 0..4095 can represent it: unwrapping detects the straddle and it must be
	// flagged rather than mapped to a broken calibration.
	spec := JointSpec{ID: 1, Name: "shoulder_pan", LowerRad: -1.91986, UpperRad: 1.91986, HasHardStop: true}
	r := newScriptedReader(map[uint32][]uint16{1: {4000, 3900, 3990, 4090, 4095, 5, 200, 600}})
	rec := NewRecorder([]JointSpec{spec})
	rec.SetHome(r)
	driveAll(rec, r, 7)

	cal := rec.Results(DefaultRecordConfig())[0]
	require.False(t, cal.OK)
	require.Equal(t, FlagSeam, cal.FlagReason)
}

func TestRecorder_NonSeamWideSpanIsContinuous(t *testing.T) {
	spec := JointSpec{ID: 1, Name: "shoulder_pan", LowerRad: -1.91986, UpperRad: 1.91986, HasHardStop: true}
	r := newScriptedReader(map[uint32][]uint16{1: {2048, 800, 1500, 2048, 2600, 3300}})
	rec := NewRecorder([]JointSpec{spec})
	rec.SetHome(r)
	driveAll(rec, r, 5)

	cal := rec.Results(DefaultRecordConfig())[0]
	require.True(t, cal.OK)
	require.Equal(t, uint16(800), cal.RawMin)
	require.Equal(t, uint16(3300), cal.RawMax)
	require.InDelta(t, 0, cal.ThetaRad(2048), 1e-9)
}

func TestRecorder_StoplessJointCentersOnHome(t *testing.T) {
	spec := JointSpec{ID: 5, Name: "wrist_roll", LowerRad: -2.74385, UpperRad: 2.84121, HasHardStop: false}
	r := newScriptedReader(map[uint32][]uint16{5: {3000, 3000, 3100, 2900}})
	rec := NewRecorder([]JointSpec{spec})
	rec.SetHome(r)
	driveAll(rec, r, 3)

	cfg := DefaultRecordConfig()
	cal := rec.Results(cfg)[0]
	require.True(t, cal.OK)
	require.InDelta(t, 0, cal.ThetaRad(3000), 1e-9) // home is the zero
	require.Equal(t, uint16(3000-cfg.NoStopCapTicks), cal.RawMin)
	require.Equal(t, uint16(3000+cfg.NoStopCapTicks), cal.RawMax)
}

func TestRecorder_UnreadJointIsFlagged(t *testing.T) {
	spec := JointSpec{ID: 3, Name: "elbow_flex", LowerRad: -1.69, UpperRad: 1.69, HasHardStop: true}
	rec := NewRecorder([]JointSpec{spec})
	rec.Sample(newScriptedReader(map[uint32][]uint16{})) // joint 3 never returns OK

	cal := rec.Results(DefaultRecordConfig())[0]
	require.False(t, cal.OK)
	require.Equal(t, FlagNoRead, cal.FlagReason)
}

func TestRecorder_SweptButNotHomedIsFlagged(t *testing.T) {
	spec := JointSpec{ID: 3, Name: "elbow_flex", LowerRad: -1.69, UpperRad: 1.69, HasHardStop: true}
	r := newScriptedReader(map[uint32][]uint16{3: {1000, 1500, 2000, 2500}})
	rec := NewRecorder([]JointSpec{spec})
	driveAll(rec, r, 4) // sweep, but SetHome never called
	require.False(t, rec.HomeSet())

	cal := rec.Results(DefaultRecordConfig())[0]
	require.False(t, cal.OK)
	require.Equal(t, FlagNoHome, cal.FlagReason)
}
