package poses

import (
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestStore_RoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "poses.toml")
	in := []Pose{
		{Slot: SlotOptions, Name: "stow", Servo: []ServoPos{{ID: 1, Raw: 2048}, {ID: 2, Raw: 1500}}},
		{Slot: SlotShare, Name: "reach", Servo: []ServoPos{{ID: 1, Raw: 1000}, {ID: 5, Raw: 3000}}},
	}
	require.NoError(t, Save(path, in))
	out, err := Load(path)
	require.NoError(t, err)
	// Load sorts by slot, so "options" precedes "share".
	require.Equal(t, []Pose{in[0], in[1]}, out)
}

func TestLoad_MissingFileIsEmpty(t *testing.T) {
	out, err := Load(filepath.Join(t.TempDir(), "none.toml"))
	require.NoError(t, err)
	require.Empty(t, out)
}

// fakeReader returns canned raw positions; an id absent from raw reads as "no read".
type fakeReader struct{ raw map[uint32]uint16 }

func (f fakeReader) ReadRaw(id uint32) (uint16, bool) {
	v, ok := f.raw[id]
	return v, ok
}

func TestStore_CaptureReadsLiveRaw(t *testing.T) {
	r := fakeReader{raw: map[uint32]uint16{1: 2048, 2: 1500, 3: 1800, 4: 2000, 5: 2500, 6: 1200}}
	s := NewStore(r, nil)

	p, err := s.Capture(SlotShare, "reach", []uint32{1, 2, 3, 4, 5, 6})
	require.NoError(t, err)
	require.Equal(t, SlotShare, p.Slot)
	require.Len(t, p.Servo, 6)

	m := s.Map()[SlotShare]
	require.Equal(t, uint16(2048), m[1])
	require.Equal(t, uint16(2500), m[5])
}

func TestStore_CaptureSkipsUnreadableServos(t *testing.T) {
	r := fakeReader{raw: map[uint32]uint16{1: 2048}} // only joint 1 reads
	s := NewStore(r, nil)
	p, err := s.Capture(SlotShare, "", []uint32{1, 2, 3})
	require.NoError(t, err)
	require.Len(t, p.Servo, 1)
}

func TestStore_CaptureNoReadsIsError(t *testing.T) {
	s := NewStore(fakeReader{raw: map[uint32]uint16{}}, nil)
	_, err := s.Capture(SlotShare, "", []uint32{1, 2})
	require.Error(t, err)
	require.Empty(t, s.All())
}

func TestStore_DeleteAndAll(t *testing.T) {
	s := NewStore(fakeReader{raw: map[uint32]uint16{1: 100}}, nil)
	_, _ = s.Capture(SlotShare, "a", []uint32{1})
	_, _ = s.Capture(SlotOptions, "b", []uint32{1})
	require.Len(t, s.All(), 2)
	s.Delete(SlotShare)
	all := s.All()
	require.Len(t, all, 1)
	require.Equal(t, SlotOptions, all[0].Slot)
}
