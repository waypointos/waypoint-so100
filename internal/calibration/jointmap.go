// Package calibration computes the SO-101 arm's raw-tick to URDF-radian map
// from measured hard stops. The per-tick scale is fixed (direct-drive joints),
// so calibration discovers only the homing offset and soft limits. Zero is
// anchored on phi: the joint's URDF lower-limit angle, which is the angle it is
// in when resting on its lower hard stop.
package calibration

import "math"

const TicksPerRev = 4096.0

// TicksPerRad converts radians to encoder ticks (fixed for direct-drive joints).
var TicksPerRad = TicksPerRev / (2 * math.Pi)

// SoftLimitMarginTicks backs the soft limits off the measured hard stops.
const SoftLimitMarginTicks = 20

// Flag reasons recorded on a JointCal. Empty means a clean calibration.
// FlagSpanMismatch still carries a usable map (the measured span just differs
// from the URDF span — usually an incomplete sweep); FlagNoRead and FlagSeam do
// NOT carry a map and must not be applied (see Mapped).
const (
	FlagSpanMismatch = "span mismatch"
	FlagNoRead       = "no read"
	FlagSeam         = "seam in workspace"
)

// JointSpec is a fixed property of the SO-101 design. LowerRad is phi.
//
// HasHardStop is false for joints that can spin freely past their URDF limits
// (wrist roll): they have no mechanical stop to record against, so calibration
// fences them to a fixed window around their resting center instead of measuring
// a "stop" that does not exist.
type JointSpec struct {
	ID          uint32
	Name        string
	LowerRad    float64
	UpperRad    float64
	HasHardStop bool
}

// SO100Joints: ids 1..6, limits from dashboard so101 URDF (joints.ts).
var SO100Joints = []JointSpec{
	{1, "shoulder_pan", -1.91986, 1.91986, true},
	{2, "shoulder_lift", -1.74533, 1.74533, true},
	{3, "elbow_flex", -1.69, 1.69, true},
	{4, "wrist_flex", -1.65806, 1.65806, true},
	{5, "wrist_roll", -2.74385, 2.84121, false}, // gripper rotation: no hard stop; snags the camera cable if seeked
	{6, "gripper", -0.17453, 1.74533, true},
}

// JointCal is the measured + derived calibration for one joint.
type JointCal struct {
	ID           uint32  `toml:"id"`
	RawMin       uint16  `toml:"raw_min"`
	RawMax       uint16  `toml:"raw_max"`
	ZeroRaw      float64 `toml:"zero_raw"`
	SoftMin      uint16  `toml:"soft_min"`
	SoftMax      uint16  `toml:"soft_max"`
	MeasuredSpan int     `toml:"measured_span"`
	ExpectedSpan int     `toml:"expected_span"`
	OK           bool    `toml:"ok"`
	FlagReason   string  `toml:"flag_reason,omitempty"` // "" when OK; else why (e.g. "span mismatch", "seam in workspace")
}

// ExpectedSpanTicks is the joint's full URDF travel in encoder ticks.
func ExpectedSpanTicks(spec JointSpec) int {
	return int(math.Round((spec.UpperRad - spec.LowerRad) * TicksPerRad))
}

// Derive computes the phi-anchored calibration from the two measured hard stops.
func Derive(spec JointSpec, rawMin, rawMax uint16, spanTolerance int) JointCal {
	measured := int(rawMax) - int(rawMin)
	expected := ExpectedSpanTicks(spec)
	ok := absInt(measured-expected) <= spanTolerance
	flag := ""
	if !ok {
		flag = FlagSpanMismatch
	}
	return JointCal{
		ID:           spec.ID,
		RawMin:       rawMin,
		RawMax:       rawMax,
		ZeroRaw:      float64(rawMin) - spec.LowerRad*TicksPerRad,
		SoftMin:      rawMin + SoftLimitMarginTicks,
		SoftMax:      rawMax - SoftLimitMarginTicks,
		MeasuredSpan: measured,
		ExpectedSpan: expected,
		OK:           ok,
		FlagReason:   flag,
	}
}

// DeriveCentered builds the calibration for a stopless joint (wrist roll). Its
// neutral resting pose is the zero, and the window is a fixed fence rather than
// two measured hard stops, so there is no span to validate: OK is always true.
func DeriveCentered(spec JointSpec, centerRaw, rawMin, rawMax uint16) JointCal {
	if rawMin > rawMax {
		rawMin, rawMax = rawMax, rawMin
	}
	return JointCal{
		ID:           spec.ID,
		RawMin:       rawMin,
		RawMax:       rawMax,
		ZeroRaw:      float64(centerRaw),
		SoftMin:      rawMin + SoftLimitMarginTicks,
		SoftMax:      rawMax - SoftLimitMarginTicks,
		MeasuredSpan: int(rawMax) - int(rawMin),
		ExpectedSpan: ExpectedSpanTicks(spec),
		OK:           true,
	}
}

// Mapped reports whether this calibration carries a usable raw->rad map. A
// joint that could not be measured (never read, or whose range straddles the
// encoder seam) has no valid zero, so it must stay uncalibrated rather than be
// applied with a garbage anchor that teleop would then drive against.
func (c JointCal) Mapped() bool {
	return c.FlagReason != FlagNoRead && c.FlagReason != FlagSeam
}

// ThetaRad maps a raw tick to a URDF joint angle using the derived zero.
func (c JointCal) ThetaRad(raw uint16) float64 {
	return (float64(raw) - c.ZeroRaw) / TicksPerRad
}

// RawFromRad is the inverse of ThetaRad, clamped to the encoder range.
func (c JointCal) RawFromRad(rad float64) uint16 {
	t := math.Round(c.ZeroRaw + rad*TicksPerRad)
	if t < 0 {
		return 0
	}
	if t > TicksPerRev-1 {
		return uint16(TicksPerRev - 1)
	}
	return uint16(t)
}

// SoftLimitsRad returns the {lower, upper} soft limits in URDF radians.
func (c JointCal) SoftLimitsRad() [2]float64 {
	return [2]float64{c.ThetaRad(c.SoftMin), c.ThetaRad(c.SoftMax)}
}

func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
