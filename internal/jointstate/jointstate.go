// Package jointstate polls servo raw positions, calibrates them to URDF radians,
// and publishes module.<id>.joints for the teleop render window.
package jointstate

import (
	"fmt"

	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
	"github.com/waypointos/waypoint-so100/internal/calibration"
)

// RawReader returns a servo's present raw position (0..4095) and whether the
// read succeeded.
type RawReader interface {
	ReadRaw(id uint32) (uint16, bool)
}

// BuildJointAngles maps raw ticks to calibrated radians; a joint with no
// calibration or a failed read is reported N/A (absent angle + reason).
func BuildJointAngles(ids []uint32, r RawReader, cals map[uint32]calibration.JointCal) *so100v1.JointAngles {
	out := &so100v1.JointAngles{}
	for _, id := range ids {
		j := &so100v1.Joint{Id: id}
		cal, hasCal := cals[id]
		raw, ok := r.ReadRaw(id)
		switch {
		case !hasCal:
			j.NaReason = "uncalibrated"
		case !ok:
			j.NaReason = "no read"
		default:
			rad := float32(cal.ThetaRad(raw))
			j.AngleRad = &rad
		}
		out.Joints = append(out.Joints, j)
	}
	return out
}

// PublishSubject is the joints telemetry subject for the given rover/module.
func PublishSubject(roverID, moduleID string) string {
	return fmt.Sprintf("waypoint.%s.module.%s.joints", roverID, moduleID)
}
