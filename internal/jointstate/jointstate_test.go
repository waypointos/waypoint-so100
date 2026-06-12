package jointstate

import (
	"testing"

	"github.com/stretchr/testify/require"
	"github.com/waypointos/waypoint-so100/internal/calibration"
)

type fakeReader struct {
	raw map[uint32]uint16
	ok  map[uint32]bool
}

func (f *fakeReader) ReadRaw(id uint32) (uint16, bool) { return f.raw[id], f.ok[id] }

// mustDeriveAt builds a JointCal for the given joint id from measured hard stops.
func mustDeriveAt(t *testing.T, id uint32, rawMin, rawMax uint16) calibration.JointCal {
	t.Helper()
	var spec calibration.JointSpec
	for _, s := range calibration.SO100Joints {
		if s.ID == id {
			spec = s
		}
	}
	require.Equal(t, id, spec.ID, "no JointSpec for id %d", id)
	return calibration.Derive(spec, rawMin, rawMax, 9999)
}

func TestBuildJointAngles_CalibratedAndNA(t *testing.T) {
	cals := map[uint32]calibration.JointCal{
		1: mustDeriveAt(t, 1, 100, 3996), // calibrated joint
		// joint 2 intentionally absent -> N/A
	}
	r := &fakeReader{raw: map[uint32]uint16{1: 2048, 2: 2048}, ok: map[uint32]bool{1: true, 2: true}}

	ja := BuildJointAngles([]uint32{1, 2}, r, cals)
	require.Len(t, ja.GetJoints(), 2)
	require.NotNil(t, ja.GetJoints()[0].AngleRad) // joint 1 has an angle
	require.Nil(t, ja.GetJoints()[1].AngleRad)    // joint 2 is N/A
	require.NotEmpty(t, ja.GetJoints()[1].GetNaReason())
}
