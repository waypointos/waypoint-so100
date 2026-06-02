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

// JointSpec is a fixed property of the SO-101 design. LowerRad is phi.
type JointSpec struct {
	ID       uint32
	Name     string
	LowerRad float64
	UpperRad float64
}

// SO100Joints: ids 1..6, limits from dashboard so101 URDF (joints.ts).
var SO100Joints = []JointSpec{
	{1, "shoulder_pan", -1.91986, 1.91986},
	{2, "shoulder_lift", -1.74533, 1.74533},
	{3, "elbow_flex", -1.69, 1.69},
	{4, "wrist_flex", -1.65806, 1.65806},
	{5, "wrist_roll", -2.74385, 2.84121},
	{6, "gripper", -0.17453, 1.74533},
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
}

// Derive computes the phi-anchored calibration from the two measured hard stops.
func Derive(spec JointSpec, rawMin, rawMax uint16, spanTolerance int) JointCal {
	measured := int(rawMax) - int(rawMin)
	expected := int(math.Round((spec.UpperRad - spec.LowerRad) * TicksPerRad))
	return JointCal{
		ID:           spec.ID,
		RawMin:       rawMin,
		RawMax:       rawMax,
		ZeroRaw:      float64(rawMin) - spec.LowerRad*TicksPerRad,
		SoftMin:      rawMin + SoftLimitMarginTicks,
		SoftMax:      rawMax - SoftLimitMarginTicks,
		MeasuredSpan: measured,
		ExpectedSpan: expected,
		OK:           absInt(measured-expected) <= spanTolerance,
	}
}

// ThetaRad maps a raw tick to a URDF joint angle using the derived zero.
func (c JointCal) ThetaRad(raw uint16) float64 {
	return (float64(raw) - c.ZeroRaw) / TicksPerRad
}

func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
