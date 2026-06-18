// Package control runs the so100 manual calibration on command and reports
// progress. Calibration is a torque-off, human-in-the-loop range recording: the
// operator sweeps every joint to its extremes while the controller polls the
// servos, then on finish it derives and persists the phi-anchored map.
package control

import (
	"sync"
	"time"

	"github.com/waypointos/waypoint-so100/internal/calibration"
)

// Publish reports calibration progress (the controller stays transport-agnostic;
// main wires this to module.so100.calibration). On the terminal "done" phase the
// wiring persists and applies the calibration.
type Publish func(cals []calibration.JointCal, phase string, activeJoint uint32)

type Controller struct {
	client   calibration.ServoClient
	pub      Publish
	cfg      calibration.RecordConfig
	sample   time.Duration // how often to poll servo positions while recording
	pubEvery time.Duration // how often to publish the live range to the tab

	mu  sync.Mutex
	rec *session // non-nil while a recording session is live
}

type session struct {
	finish chan bool // true => derive + save, false => abort
}

func New(client calibration.ServoClient, pub Publish) *Controller {
	return &Controller{
		client:   client,
		pub:      pub,
		cfg:      calibration.DefaultRecordConfig(),
		sample:   33 * time.Millisecond,
		pubEvery: 150 * time.Millisecond,
	}
}

// StartRecording begins a manual range-recording session: it disables torque on
// every joint so the operator can backdrive the arm by hand, then polls and
// reports the swept range until FinishRecording or Abort. A second call while a
// session is live is ignored.
func (c *Controller) StartRecording() {
	c.mu.Lock()
	if c.rec != nil {
		c.mu.Unlock()
		return
	}
	sess := &session{finish: make(chan bool, 1)}
	c.rec = sess
	c.mu.Unlock()

	for _, spec := range calibration.SO100Joints {
		_ = c.client.DisableTorque(spec.ID)
	}
	go c.record(sess)
}

// FinishRecording ends a live session, deriving and persisting the calibration
// from the swept ranges. A no-op if nothing is recording.
func (c *Controller) FinishRecording() { c.signal(true) }

// Abort ends a live session and discards it (torque stays off). A no-op if
// nothing is recording.
func (c *Controller) Abort() { c.signal(false) }

func (c *Controller) signal(save bool) {
	c.mu.Lock()
	sess := c.rec
	c.mu.Unlock()
	if sess == nil {
		return
	}
	select {
	case sess.finish <- save:
	default: // already signalled
	}
}

func (c *Controller) record(sess *session) {
	rec := calibration.NewRecorder(calibration.SO100Joints)
	sampleT := time.NewTicker(c.sample)
	pubT := time.NewTicker(c.pubEvery)
	defer sampleT.Stop()
	defer pubT.Stop()

	c.pub(rec.Live(), "recording", 0)
	for {
		select {
		case save := <-sess.finish:
			c.mu.Lock()
			c.rec = nil
			c.mu.Unlock()
			if !save {
				c.pub(rec.Live(), "aborted", 0)
				return
			}
			results := rec.Results(c.cfg)
			c.applyFences(results)
			c.pub(results, "done", 0)
			return
		case <-sampleT.C:
			rec.Sample(c.client)
		case <-pubT.C:
			c.pub(rec.Live(), "recording", 0)
		}
	}
}

// applyFences writes each calibrated joint's soft limits to the servo's hardware
// angle-limit registers as a backstop, so a runaway goal cannot drive a joint
// past its measured range even if a software clamp is bypassed. Joints that did
// not calibrate (no read / seam) are left untouched.
func (c *Controller) applyFences(results []calibration.JointCal) {
	for _, cal := range results {
		if cal.FlagReason == "no read" || cal.FlagReason == "seam in workspace" {
			continue
		}
		_ = c.client.SetAngleLimits(cal.ID, cal.SoftMin, cal.SoftMax)
	}
}
