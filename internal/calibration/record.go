package calibration

import "math"

// Recorder tracks each joint's swept range during a manual, torque-off
// calibration: the operator backdrives every joint to both extremes while
// Sample is polled in a loop. It is the robust alternative to powered hard-stop
// seeking — the human supplies the "find the stop" intelligence, so nothing is
// rammed and the weight-bearing shoulder joints are never driven.
//
// Sample unwraps the encoder seam (the 0/4095 boundary) by accumulating signed
// per-read deltas, so a joint whose travel happens to span the seam still yields
// a correct continuous range. The raw tick observed at each extreme is retained
// for the phi-anchored Derive. If a joint's working range genuinely straddles
// the seam, no single linear raw->rad map over 0..4095 can represent it; Results
// detects that and flags the joint instead of emitting a broken calibration.
type Recorder struct {
	specs []JointSpec
	tr    map[uint32]*track
}

type track struct {
	started            bool
	startRaw           uint16  // first reading; the resting zero for stopless joints
	lastRaw            uint16  // previous reading, for delta/seam unwrapping
	cont               float64 // continuous (unwrapped) position, ticks relative to start
	contMin, contMax   float64
	rawAtMin, rawAtMax uint16 // raw tick at the continuous min/max extreme
}

// SeamSlackTicks is how far the direct raw-tick difference between the two
// extremes may diverge from the true (unwrapped) span before the range is judged
// to straddle the encoder seam.
const SeamSlackTicks = 200

func NewRecorder(specs []JointSpec) *Recorder {
	tr := make(map[uint32]*track, len(specs))
	for _, s := range specs {
		tr[s.ID] = &track{}
	}
	return &Recorder{specs: specs, tr: tr}
}

// Sample reads every joint once and folds each reading into its tracked range.
// A joint that fails to read is skipped for this tick (no reading, no update).
func (r *Recorder) Sample(c ServoClient) {
	for _, s := range r.specs {
		rd, err := c.Read(s.ID)
		if err != nil || !rd.OK {
			continue
		}
		t := r.tr[s.ID]
		if !t.started {
			t.started = true
			t.startRaw = rd.PositionRaw
			t.lastRaw = rd.PositionRaw
			t.rawAtMin, t.rawAtMax = rd.PositionRaw, rd.PositionRaw
			continue
		}
		d := int(rd.PositionRaw) - int(t.lastRaw)
		switch {
		case d > int(TicksPerRev)/2:
			d -= int(TicksPerRev)
		case d < -int(TicksPerRev)/2:
			d += int(TicksPerRev)
		}
		t.cont += float64(d)
		t.lastRaw = rd.PositionRaw
		if t.cont < t.contMin {
			t.contMin, t.rawAtMin = t.cont, rd.PositionRaw
		}
		if t.cont > t.contMax {
			t.contMax, t.rawAtMax = t.cont, rd.PositionRaw
		}
	}
}

// Live returns the in-progress ranges for display while recording: the raw tick
// span swept so far per joint, all not-yet-OK. Joints not yet read are omitted.
func (r *Recorder) Live() []JointCal {
	out := make([]JointCal, 0, len(r.specs))
	for _, s := range r.specs {
		t := r.tr[s.ID]
		if !t.started {
			continue
		}
		lo, hi := orderRaw(t.rawAtMin, t.rawAtMax)
		out = append(out, JointCal{ID: s.ID, RawMin: lo, RawMax: hi})
	}
	return out
}

// Results derives the final calibration for every joint from the swept ranges.
// Jointed limbs are phi-anchored from their two extremes; a stopless joint
// (wrist roll) is centered on its resting start and fenced to a fixed window.
// A joint never read, or one whose range straddles the encoder seam, is returned
// not-OK with a flag instead of a broken map.
func (r *Recorder) Results(cfg RecordConfig) []JointCal {
	out := make([]JointCal, 0, len(r.specs))
	for _, s := range r.specs {
		out = append(out, r.result(s, cfg))
	}
	return out
}

func (r *Recorder) result(s JointSpec, cfg RecordConfig) JointCal {
	t := r.tr[s.ID]
	if !t.started {
		return JointCal{ID: s.ID, FlagReason: FlagNoRead}
	}

	if !s.HasHardStop {
		lo := clampTick(int(t.startRaw) - cfg.NoStopCapTicks)
		hi := clampTick(int(t.startRaw) + cfg.NoStopCapTicks)
		return DeriveCentered(s, t.startRaw, lo, hi)
	}

	trueSpan := t.contMax - t.contMin
	directSpan := absInt(int(t.rawAtMax) - int(t.rawAtMin))
	if math.Abs(trueSpan-float64(directSpan)) > SeamSlackTicks {
		return JointCal{ID: s.ID, FlagReason: FlagSeam}
	}
	lo, hi := orderRaw(t.rawAtMin, t.rawAtMax)
	return Derive(s, lo, hi, cfg.SpanTolerance)
}

func orderRaw(a, b uint16) (lo, hi uint16) {
	if a > b {
		return b, a
	}
	return a, b
}
