package calibration

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRawFromRad_RoundTripsWithThetaRad(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	cal := Derive(spec, 820, 3170, 80)
	for _, raw := range []uint16{900, 1500, 2048, 3000} {
		got := cal.RawFromRad(cal.ThetaRad(raw))
		require.InDelta(t, raw, got, 1)
	}
}

func TestRawFromRad_ClampsToEncoderRange(t *testing.T) {
	spec := JointSpec{ID: 1, Name: "shoulder_pan", LowerRad: -1.91986, UpperRad: 1.91986}
	cal := Derive(spec, 100, 3996, 9999)
	require.Equal(t, uint16(0), cal.RawFromRad(-100))
	require.Equal(t, uint16(4095), cal.RawFromRad(100))
}

func TestSoftLimitsRad_BracketsZero(t *testing.T) {
	spec := JointSpec{ID: 2, Name: "shoulder_lift", LowerRad: -1.74533, UpperRad: 1.74533}
	cal := Derive(spec, 820, 3170, 80)
	lim := cal.SoftLimitsRad()
	require.Less(t, lim[0], lim[1])
	require.InDelta(t, cal.ThetaRad(cal.SoftMin), lim[0], 1e-9)
	require.InDelta(t, cal.ThetaRad(cal.SoftMax), lim[1], 1e-9)
}
