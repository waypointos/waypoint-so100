package calibration

// RecordConfig tunes the manual range-recording calibration. Defaults are
// conservative; tune on hardware.
type RecordConfig struct {
	// SpanTolerance is how far a jointed limb's measured hard-stop span may
	// differ from its URDF span before it is flagged as a span mismatch (ticks).
	SpanTolerance int
	// NoStopCapTicks is the soft-limit half-window each way from the resting
	// center for a stopless joint (wrist roll), which has no hard stop to
	// measure. 512 ticks = 45 deg each way = a 90 deg window; the wrist roll is
	// fenced here and never power-driven, so the camera cable is never wound
	// past this window.
	NoStopCapTicks int
}

func DefaultRecordConfig() RecordConfig {
	return RecordConfig{
		SpanTolerance:  60,
		NoStopCapTicks: 512,
	}
}

func clampTick(t int) uint16 {
	if t < 0 {
		return 0
	}
	if t > 4095 {
		return 4095
	}
	return uint16(t)
}
