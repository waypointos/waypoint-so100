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

// StateReader returns a servo's full state for the MOTOR DETAIL card; an error
// marks that servo absent (ok=false) rather than poisoning the row with zeros.
type StateReader interface {
	ReadServoState(id uint32) (*so100v1.ServoState, error)
}

// BuildServoStats collects full per-servo telemetry for the Arm tab. A failed
// read becomes an ok=false entry with every scalar absent, so the panel can
// render N/A for that servo without inventing values.
func BuildServoStats(ids []uint32, r StateReader) *so100v1.ServoStats {
	out := &so100v1.ServoStats{}
	for _, id := range ids {
		st, err := r.ReadServoState(id)
		if err != nil || st == nil {
			st = &so100v1.ServoState{ServoId: id}
		}
		out.Servos = append(out.Servos, st)
	}
	return out
}

// PublishSubject is the joints telemetry subject for the given rover/module.
func PublishSubject(roverID, moduleID string) string {
	return fmt.Sprintf("waypoint.%s.module.%s.joints", roverID, moduleID)
}
