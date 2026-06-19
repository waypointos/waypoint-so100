package main

import (
	"context"
	"log/slog"
	"os"
	"time"

	natsgo "github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	waypointv1 "github.com/waypointos/waypoint/protocol/gen/go/messages"
	"github.com/waypointos/waypoint/sdk/wpmodule"

	"github.com/waypointos/waypoint-so100/internal/armserver"
	"github.com/waypointos/waypoint-so100/internal/calibration"
	"github.com/waypointos/waypoint-so100/internal/config"
	"github.com/waypointos/waypoint-so100/internal/control"
	"github.com/waypointos/waypoint-so100/internal/ik"
	"github.com/waypointos/waypoint-so100/internal/jointstate"
	"github.com/waypointos/waypoint-so100/internal/servobus"
	"github.com/waypointos/waypoint-so100/internal/teleop"
	so100v1 "github.com/waypointos/waypoint-so100/protocol/gen/go"
)

func main() {
	slog.SetDefault(slog.New(slog.NewTextHandler(os.Stderr, nil)))
	err := wpmodule.Run(context.Background(), wpmodule.Options{ID: "so100"}, setup)
	if err != nil {
		slog.Error(err.Error())
		os.Exit(1)
	}
}

// setup wires the so100 module on the SDK runtime. The SDK owns connect, creds,
// sd_notify, health, and stats; this recreates the module's own surfaces:
// calibration, the private command, teleop, the joints publisher, and the
// standard arm component. Goroutines started here stop on m.Done(), before
// the SDK drains the connection.
func setup(m *wpmodule.M) error {
	cfg, err := config.Load(os.Getenv("WAYPOINT_MODULE_CONFIG"))
	if err != nil {
		return err
	}

	svc := m.Servo()
	sv := servobus.New(svc)

	cals := map[uint32]calibration.JointCal{}

	// Joint name + order from the SO-101 design (ids 1..6).
	names := map[uint32]string{}
	order := make([]uint32, 0, len(calibration.SO100Joints))
	for _, spec := range calibration.SO100Joints {
		names[spec.ID] = spec.Name
		order = append(order, spec.ID)
	}
	// Reader bridges the so100 raw width; writer is the SDK client's native
	// waypoint.v1 goal path.
	armSrv := armserver.New(sv, svc, cals, names, order)

	// applyCals refreshes the live calibration view shared by the joints
	// publisher and teleop loop, and the arm server's own snapshot.
	applyCals := func(loaded []calibration.JointCal) {
		next := map[uint32]calibration.JointCal{}
		for _, c := range loaded {
			next[c.ID] = c
			cals[c.ID] = c
		}
		armSrv.SetCalibration(next)
	}

	// Calibration progress publisher keeps its private subject; on "done" it
	// persists and refreshes the live calibration.
	calSubject := m.Subject("calibration")
	publish := func(progress []calibration.JointCal, phase string, active uint32) {
		st := &so100v1.CalibrationState{T: timestamppb.Now(), Phase: phase, ActiveJoint: active}
		for _, c := range progress {
			jc := &so100v1.JointCalibration{
				Id: c.ID, Ok: c.OK, FlagReason: c.FlagReason,
				RawMin: proto.Uint32(uint32(c.RawMin)), RawMax: proto.Uint32(uint32(c.RawMax)),
				ZeroRaw: proto.Float64(c.ZeroRaw),
				SoftMin: proto.Uint32(uint32(c.SoftMin)), SoftMax: proto.Uint32(uint32(c.SoftMax)),
				MeasuredSpan: proto.Int32(int32(c.MeasuredSpan)), ExpectedSpan: proto.Int32(int32(c.ExpectedSpan)),
			}
			st.Joints = append(st.Joints, jc)
		}
		body, _ := proto.Marshal(st)
		_ = m.Publish(calSubject, body)
		if phase == "done" {
			// Persist and apply only joints that produced a usable map; a joint
			// that failed (no read / seam) must stay uncalibrated rather than
			// poison teleop with a garbage zero anchor.
			mapped := make([]calibration.JointCal, 0, len(progress))
			for _, c := range progress {
				if c.Mapped() {
					mapped = append(mapped, c)
				}
			}
			_ = calibration.Save(cfg.StatePath, mapped)
			applyCals(mapped)
		}
	}

	ctrl := control.New(sv, publish)

	// Private command handler: run calibration in a goroutine so the callback
	// returns promptly. Coexists with the standard arm.cmd the SDK serves.
	if _, err := m.Subscribe(m.Subject("command"), func(msg *natsgo.Msg) {
		var cmd so100v1.ArmCommand
		if proto.Unmarshal(msg.Data, &cmd) != nil {
			return
		}
		switch {
		case cmd.GetRunCalibration():
			ctrl.StartRecording()
		case cmd.GetFinishCalibration():
			ctrl.FinishRecording()
		case cmd.GetAbort():
			ctrl.Abort()
		}
	}); err != nil {
		return err
	}

	// Publish persisted calibration once at boot so the tab shows last state.
	if loaded, err := calibration.Load(cfg.StatePath); err == nil {
		publish(loaded, "idle", 0)
		applyCals(loaded)
	}

	// Teleop: the SDK relays the gamepad stream; convert it to the so100 wire
	// shape (identical fields) for the IK loop.
	loop := teleop.NewLoop(teleop.LoopConfig{
		StaleAfter: 150 * time.Millisecond,
		MaxLinear:  0.15, MaxPitch: 1.0, RampPerTick: 0.02, Dt: 0.02,
	}, sv, cals, ik.SO100Kinematics())

	if _, err := m.TeleopInput(func(s *waypointv1.GamepadSnapshot) {
		loop.SetInput(&so100v1.GamepadSnapshot{
			Axes: s.GetAxes(), Buttons: s.GetButtons(), Triggers: s.GetTriggers(),
		}, time.Now())
	}); err != nil {
		return err
	}

	// Standard arm component API (floor over the same servo path). A standard
	// stop must freeze teleop motion too, not just re-latch goals.
	armSrv.SetHalt(loop.Halt)
	if _, err := m.ServeArm(armSrv); err != nil {
		return err
	}

	go loop.Run(m.Done())

	// 33 ms joint-angle publisher feeds the private joints subject (panel).
	go func() {
		t := time.NewTicker(33 * time.Millisecond)
		defer t.Stop()
		for {
			select {
			case <-m.Done():
				return
			case <-t.C:
				// Yield the servo bus to the calibration recorder while it is
				// sweeping; two readers at ~30 Hz starve the long wrist/gripper
				// chain and surface as "no read".
				if ctrl.Recording() {
					continue
				}
				ja := jointstate.BuildJointAngles([]uint32{1, 2, 3, 4, 5, 6}, sv, cals)
				if b, err := proto.Marshal(ja); err == nil {
					_ = m.Publish(m.Subject("joints"), b)
				}
				loop.SetJointEstimate(jointEstimateFrom(ja))
			}
		}
	}()

	return nil
}

// jointEstimateFrom converts published JointAngles (ids 1..5) into the loop's
// [5]float64 seed; an absent (N/A) angle leaves that joint at the prior value 0.
func jointEstimateFrom(ja *so100v1.JointAngles) [5]float64 {
	var q [5]float64
	for _, j := range ja.GetJoints() {
		id := j.GetId()
		if id >= 1 && id <= 5 && j.AngleRad != nil {
			q[id-1] = float64(j.GetAngleRad())
		}
	}
	return q
}
