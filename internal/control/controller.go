// Package control runs the so100 calibration on command and reports progress.
package control

import (
	"github.com/waypointos/waypoint-so100/internal/calibration"
)

// Publish reports calibration progress (the controller stays transport-agnostic;
// main wires this to module.so100.calibration).
type Publish func(cals []calibration.JointCal, phase string, activeJoint uint32)

type Controller struct {
	client calibration.ServoClient
	pub    Publish
	cfg    calibration.CalibrateConfig
}

func New(client calibration.ServoClient, pub Publish) *Controller {
	return &Controller{client: client, pub: pub, cfg: calibration.DefaultCalibrateConfig()}
}

// RunCalibration calibrates joints 1..6 in order, publishing after each. A
// joint that fails (seam/timeout) is recorded not-OK and the run continues.
func (c *Controller) RunCalibration() {
	results := make([]calibration.JointCal, 0, len(calibration.SO100Joints))
	for _, spec := range calibration.SO100Joints {
		c.pub(results, "running", spec.ID)
		cal, err := calibration.CalibrateJoint(c.client, spec, c.cfg)
		if err != nil {
			cal = calibration.JointCal{ID: spec.ID, OK: false}
		}
		results = append(results, cal)
	}
	c.pub(results, "done", 0)
}
