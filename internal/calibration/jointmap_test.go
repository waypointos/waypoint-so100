package calibration

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestDerive_AnchorsZeroOnLowerHardStop(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	// Measured stops roughly symmetric; span close to expected (diff 74 ticks).
	cal := Derive(spec, 820, 3170, 80)
	// At the lower hard stop the joint must read its URDF lower limit.
	require.InDelta(t, spec.LowerRad, cal.ThetaRad(820), 1e-3)
	require.True(t, cal.OK)
	require.Equal(t, uint16(820+SoftLimitMarginTicks), cal.SoftMin)
	require.Equal(t, uint16(3170-SoftLimitMarginTicks), cal.SoftMax)
}

func TestDerive_FlagsSpanMismatch(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	// Stops far too narrow (e.g. a slipped horn): measured span << expected.
	cal := Derive(spec, 1500, 1900, 40)
	require.False(t, cal.OK)
}

func TestExpectedSpan_MatchesEncoderResolution(t *testing.T) {
	spec := JointSpec{ID: 2, LowerRad: -1.74533, UpperRad: 1.74533}
	cal := Derive(spec, 0, 4095, 9999)
	want := int(math.Round((spec.UpperRad - spec.LowerRad) * TicksPerRad))
	require.Equal(t, want, cal.ExpectedSpan)
}
