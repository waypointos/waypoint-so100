// Package calibration computes the SO-101 arm's raw-tick to URDF-radian map.
// The per-tick scale is fixed (direct-drive joints), so calibration discovers
// only the homing offset and soft limits. Zero is anchored on a captured home
// pose: the operator moves the arm to its neutral (URDF-zero) configuration and
// the raw tick read there becomes 0 rad. The soft limits come from a separate
// manual sweep of the reachable range. Anchoring on the home pose (rather than a
// hard stop) keeps the zero correct even when the chassis limits a joint's
// travel below its URDF range or makes it asymmetric.
package calibration

import "math"

const TicksPerRev = 4096.0

// TicksPerRad converts radians to encoder ticks (fixed for direct-drive joints).
var TicksPerRad = TicksPerRev / (2 * math.Pi)

// SoftLimitMarginTicks backs the soft limits off the swept range extremes.
const SoftLimitMarginTicks = 20

// MinRangeTicks is the smallest reachable span (rawMax-rawMin) a hard-stopped
// joint must show; below this the sweep was too small to admit valid soft limits
// (~18 deg).
const MinRangeTicks = 200

// Flag reasons recorded on a JointCal. Empty means a clean, applicable
// calibration (see OK). Every non-empty reason means the joint must NOT be
// applied: it has either no valid zero (FlagNoRead, FlagNoHome, FlagSeam) or
// unusable limits (FlagHomeOutOfRange, FlagRangeTooSmall).
const (
	FlagNoRead         = "no read"
	FlagNoHome         = "no home pose"
	FlagHomeOutOfRange = "home outside swept range"
	FlagRangeTooSmall  = "range too small"
	FlagSeam           = "seam in workspace"
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
	FlagReason   string  `toml:"flag_reason,omitempty"` // "" when OK; else why (see Flag* constants)
}

// ExpectedSpanTicks is the joint's full URDF travel in encoder ticks.
func ExpectedSpanTicks(spec JointSpec) int {
	return int(math.Round((spec.UpperRad - spec.LowerRad) * TicksPerRad))
}

// DeriveHome computes a hard-stopped joint's calibration from the captured home
// pose (the zero) and the swept reachable range (the soft limits). The zero does
// not assume the swept extremes sit at the URDF limits, so a chassis-limited or
// asymmetric range still yields a correct zero. The joint is flagged (and left
// un-OK, hence not applied) if the sweep was too small to admit soft limits or
// if the home pose falls outside the swept range — both signal a bad session.
func DeriveHome(spec JointSpec, homeRaw, rawMin, rawMax uint16) JointCal {
	if rawMin > rawMax {
		rawMin, rawMax = rawMax, rawMin
	}
	measured := int(rawMax) - int(rawMin)
	ok := true
	flag := ""
	switch {
	case measured < MinRangeTicks:
		ok, flag = false, FlagRangeTooSmall
	case homeRaw < rawMin || homeRaw > rawMax:
		ok, flag = false, FlagHomeOutOfRange
	}
	return JointCal{
		ID:           spec.ID,
		RawMin:       rawMin,
		RawMax:       rawMax,
		ZeroRaw:      float64(homeRaw),
		SoftMin:      rawMin + SoftLimitMarginTicks,
		SoftMax:      rawMax - SoftLimitMarginTicks,
		MeasuredSpan: measured,
		ExpectedSpan: ExpectedSpanTicks(spec),
		OK:           ok,
		FlagReason:   flag,
	}
}

// DeriveCentered builds the calibration for a stopless joint (wrist roll). Its
// home pose is the zero, and the window is a fixed fence rather than two measured
// hard stops, so there is no range to validate: OK is always true.
func DeriveCentered(spec JointSpec, homeRaw, rawMin, rawMax uint16) JointCal {
	if rawMin > rawMax {
		rawMin, rawMax = rawMax, rawMin
	}
	return JointCal{
		ID:           spec.ID,
		RawMin:       rawMin,
		RawMax:       rawMax,
		ZeroRaw:      float64(homeRaw),
		SoftMin:      rawMin + SoftLimitMarginTicks,
		SoftMax:      rawMax - SoftLimitMarginTicks,
		MeasuredSpan: int(rawMax) - int(rawMin),
		ExpectedSpan: ExpectedSpanTicks(spec),
		OK:           true,
	}
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
