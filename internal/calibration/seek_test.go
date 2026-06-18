package calibration

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// fakePlant simulates one servo: position eases toward goal but cannot pass the
// hard stop; current spikes once it is pressing the stop. Optionally injects an
// encoder-seam jump at a given commanded position.
type fakePlant struct {
	pos      uint16
	stop     uint16 // hard stop position (seek + direction)
	seamAt   int    // commanded goal at which to inject a near-4096 jump; <0 = never
	lastGoal uint16
}

func (p *fakePlant) SetMode(uint32, uint32) error                { return nil }
func (p *fakePlant) SetTorqueLimit(uint32, uint16) error         { return nil }
func (p *fakePlant) SetOvercurrentLimit(uint32, uint16) error    { return nil }
func (p *fakePlant) SetAngleLimits(uint32, uint16, uint16) error { return nil }
func (p *fakePlant) SetGoalSpeed(uint32, uint16) error           { return nil }
func (p *fakePlant) EnableTorque(uint32) error                   { return nil }
func (p *fakePlant) DisableTorque(uint32) error                  { return nil }
func (p *fakePlant) SetGoalPosition(_ uint32, raw uint16) error  { p.lastGoal = raw; return nil }
func (p *fakePlant) Read(uint32) (ServoReading, error) {
	if p.seamAt >= 0 && int(p.lastGoal) >= p.seamAt {
		// Wrap across the seam: the encoder reading jumps to the opposite end,
		// a near-4096 discontinuity from the previous mid-range reading.
		p.pos = uint16((int(p.pos) + 2048) % 4096)
		return ServoReading{PositionRaw: p.pos, CurrentRaw: 50, OK: true}, nil
	}
	if p.lastGoal >= p.stop {
		p.pos = p.stop
		return ServoReading{PositionRaw: p.pos, CurrentRaw: 900, OK: true}, nil // pressing stop
	}
	p.pos = p.lastGoal
	return ServoReading{PositionRaw: p.pos, CurrentRaw: 50, OK: true}, nil
}

func defaultSeekCfg(dir int) SeekConfig {
	return SeekConfig{
		Direction: dir, StepTicks: 30, MovingSpeed: 200, CurrentLimit: 500,
		FollowErrorTicks: 80, ProgressTicks: 5, PlateauReads: 3,
		MaxTravelTicks: 4096, SeamJumpTicks: 1000,
	}
}

func TestSeek_FindsHardStopOnCurrentSpike(t *testing.T) {
	p := &fakePlant{pos: 2048, stop: 3000, seamAt: -1}
	res := SeekHardStop(p, 3, defaultSeekCfg(+1))
	require.Equal(t, SeekOK, res.Reason)
	require.InDelta(t, 3000, int(res.RawStop), 60) // within a step of the stop
}

func TestSeek_DetectsSeamInWorkspace(t *testing.T) {
	p := &fakePlant{pos: 2048, stop: 4095, seamAt: 2200}
	res := SeekHardStop(p, 3, defaultSeekCfg(+1))
	require.Equal(t, SeekSeam, res.Reason)
}

func TestSeek_TimesOutWhenNoStop(t *testing.T) {
	// stop beyond MaxTravel and never reached; current stays low.
	p := &fakePlant{pos: 0, stop: 65000, seamAt: -1}
	cfg := defaultSeekCfg(+1)
	cfg.MaxTravelTicks = 500
	res := SeekHardStop(p, 3, cfg)
	require.Equal(t, SeekTimeout, res.Reason)
}
