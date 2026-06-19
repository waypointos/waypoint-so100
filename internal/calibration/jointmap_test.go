package calibration

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestDeriveHome_AnchorsZeroOnHomePose(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	// Home captured at 2000; swept range 820..3170 around it.
	cal := DeriveHome(spec, 2000, 820, 3170)
	require.True(t, cal.OK)
	require.InDelta(t, 0, cal.ThetaRad(2000), 1e-9) // home reads as 0 rad
	require.Equal(t, uint16(820+SoftLimitMarginTicks), cal.SoftMin)
	require.Equal(t, uint16(3170-SoftLimitMarginTicks), cal.SoftMax)
}

func TestDeriveHome_ZeroIsIndependentOfRangeSize(t *testing.T) {
	spec := JointSpec{ID: 1, Name: "shoulder_pan", LowerRad: -1.91986, UpperRad: 1.91986}
	// A chassis-limited pan reaches only a narrow, asymmetric range, but the home
	// pose still fixes the zero: home reads 0 regardless of how far it travels.
	cal := DeriveHome(spec, 2050, 1165, 2925)
	require.True(t, cal.OK)
	require.InDelta(t, 0, cal.ThetaRad(2050), 1e-9)
}

func TestDeriveHome_FlagsHomeOutsideRange(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	// Home below the swept range: the operator never swept through neutral.
	cal := DeriveHome(spec, 700, 820, 3170)
	require.False(t, cal.OK)
	require.Equal(t, FlagHomeOutOfRange, cal.FlagReason)
}

func TestDeriveHome_FlagsRangeTooSmall(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	cal := DeriveHome(spec, 2000, 1980, 2030) // 50-tick sweep, below MinRangeTicks
	require.False(t, cal.OK)
	require.Equal(t, FlagRangeTooSmall, cal.FlagReason)
}

func TestExpectedSpan_MatchesEncoderResolution(t *testing.T) {
	spec := JointSpec{ID: 2, LowerRad: -1.74533, UpperRad: 1.74533}
	cal := DeriveHome(spec, 2048, 0, 4095)
	want := int(math.Round((spec.UpperRad - spec.LowerRad) * TicksPerRad))
	require.Equal(t, want, cal.ExpectedSpan)
}
