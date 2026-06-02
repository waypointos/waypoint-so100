package calibration

import (
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestStore_RoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "calibration.toml")
	in := []JointCal{
		{ID: 1, RawMin: 100, RawMax: 4000, ZeroRaw: 2048, SoftMin: 120, SoftMax: 3980, MeasuredSpan: 3900, ExpectedSpan: 3900, OK: true},
		{ID: 2, RawMin: 820, RawMax: 3170, ZeroRaw: 1958, SoftMin: 840, SoftMax: 3150, MeasuredSpan: 2350, ExpectedSpan: 2275, OK: false},
	}
	require.NoError(t, Save(path, in))
	out, err := Load(path)
	require.NoError(t, err)
	require.Equal(t, in, out)
}

func TestLoad_MissingFileIsEmpty(t *testing.T) {
	out, err := Load(filepath.Join(t.TempDir(), "none.toml"))
	require.NoError(t, err)
	require.Empty(t, out)
}
