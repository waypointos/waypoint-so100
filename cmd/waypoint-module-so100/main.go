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
	"github.com/waypointos/waypoint-so100/internal/poses"
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
	// publisher and teleop loop, and the arm server's own snapshot. Only OK
	// joints are applied: a flagged joint must stay uncalibrated rather than be
	// driven against a bad zero, regardless of how it reached here.
	applyCals := func(loaded []calibration.JointCal) {
		next := map[uint32]calibration.JointCal{}
		for _, c := range loaded {
			if !c.OK {
				continue
			}
			next[c.ID] = c
			cals[c.ID] = c
		}
		armSrv.SetCalibration(next)
	}

	// Calibration progress publisher keeps its private subject; on "done" it
	// persists and refreshes the live calibration.
	calSubject := m.Subject("calibration")
	publish := func(progress []calibration.JointCal, phase string, homeSet bool) {
		st := &so100v1.CalibrationState{T: timestamppb.Now(), Phase: phase, HomeSet: homeSet}
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
			// Persist and apply only joints that calibrated cleanly; a flagged
			// joint (no read / no home / home out of range / seam) must stay
			// uncalibrated rather than poison teleop with a bad zero anchor.
			ok := make([]calibration.JointCal, 0, len(progress))
			for _, c := range progress {
				if c.OK {
					ok = append(ok, c)
				}
			}
			_ = calibration.Save(cfg.StatePath, ok)
			applyCals(ok)
		}
	}

	ctrl := control.New(sv, publish)

	// Teleop: the SDK relays the gamepad stream; convert it to the so100 wire
	// shape (identical fields) for the IK loop. Created before the command
	// handler so torque and pose-recall commands can drive it.
	loop := teleop.NewLoop(teleop.LoopConfig{
		StaleAfter: 150 * time.Millisecond,
		// Rates are tune-on-hardware; opened up from the initial conservative set
		// once the reference-reseed jitter (SetJointEstimate gating) was fixed.
		// Smoothness now comes from the servos' control-loop tuning applied once
		// at startup (low position-loop P + high acceleration; see applyServoTuning
		// and config.ServoTuning), so the loop just streams goal positions at max
		// speed instead of re-capping GOAL_SPEED every tick.
		MaxLinear: 0.25, MaxPitch: 1.5, MaxPan: 1.0, MaxRoll: 1.5, MaxGrip: 2.0, RampPerTick: 0.04, PanRampPerTick: 0.1, Dt: 0.02,
		// Pose recall glides to the saved pose at a fixed cap (~1.2 rad/s) instead
		// of the servos' default max speed. Button indices are config-driven; the
		// loop logs every pressed index at Debug so they can be confirmed on
		// hardware (the SDK's buttons[] layout is not the raw W3C one).
		RecallSpeed:   800,
		ShareButton:   cfg.ShareButton,
		OptionsButton: cfg.OptionsButton,
	}, sv, cals, ik.SO100Kinematics())

	// Pose store + publisher: each slot binds raw servo positions captured by hand
	// (torque off). The poses subject tells the Arm tab and teleop legend which
	// slots are assigned; the loop reads the live set for gamepad/button recall.
	posesStore := poses.NewStore(sv, nil)
	posesSubject := m.Subject("poses")
	publishPoses := func() {
		assigned := map[string]poses.Pose{}
		for _, p := range posesStore.All() {
			assigned[p.Slot] = p
		}
		st := &so100v1.PoseState{}
		for _, slot := range poses.Slots {
			p, ok := assigned[slot]
			st.Slots = append(st.Slots, &so100v1.PoseSlot{Slot: slot, Name: p.Name, Assigned: ok})
		}
		if b, err := proto.Marshal(st); err == nil {
			_ = m.Publish(posesSubject, b)
		}
		loop.SetPoses(posesStore.Map())
	}

	// setTorqueAll holds (true) or releases (false) every joint, so the operator
	// can pose the arm by hand (off) then lock it (on) from the Arm tab.
	setTorqueAll := func(on bool) {
		for _, spec := range calibration.SO100Joints {
			_ = sv.SetTorqueEnable(spec.ID, on)
		}
	}

	// Private command handler: run calibration in a goroutine so the callback
	// returns promptly. Coexists with the standard arm.cmd the SDK serves. The
	// type-switch (not a bool switch) is needed to read set_torque's false case.
	if _, err := m.Subscribe(m.Subject("command"), func(msg *natsgo.Msg) {
		var cmd so100v1.ArmCommand
		if proto.Unmarshal(msg.Data, &cmd) != nil {
			return
		}
		switch a := cmd.GetAction().(type) {
		case *so100v1.ArmCommand_RunCalibration:
			ctrl.StartRecording()
		case *so100v1.ArmCommand_SetHome:
			ctrl.SetHome()
		case *so100v1.ArmCommand_FinishCalibration:
			ctrl.FinishRecording()
		case *so100v1.ArmCommand_Abort:
			ctrl.Abort()
		case *so100v1.ArmCommand_SetTorque:
			setTorqueAll(a.SetTorque)
		case *so100v1.ArmCommand_CapturePose:
			if _, err := posesStore.Capture(a.CapturePose.GetSlot(), a.CapturePose.GetName(), order); err != nil {
				slog.Warn("pose capture failed", "slot", a.CapturePose.GetSlot(), "err", err)
				return
			}
			_ = poses.Save(cfg.PosesPath, posesStore.All())
			publishPoses()
		case *so100v1.ArmCommand_DeletePose:
			posesStore.Delete(a.DeletePose)
			_ = poses.Save(cfg.PosesPath, posesStore.All())
			publishPoses()
		case *so100v1.ArmCommand_RecallPose:
			go loop.Recall(a.RecallPose)
		}
	}); err != nil {
		return err
	}

	// Publish persisted calibration once at boot so the tab shows last state.
	if loaded, err := calibration.Load(cfg.StatePath); err == nil {
		publish(loaded, "idle", false)
		applyCals(loaded)
	}

	// Load persisted poses and publish the slot state once at boot.
	if loaded, err := poses.Load(cfg.PosesPath); err == nil {
		for _, p := range loaded {
			posesStore.Set(p)
		}
	}
	publishPoses()

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

	// Apply the one-time servo control-loop tuning once the bus is reachable, so
	// streamed goal positions glide instead of dart-and-stall (the teleop jitter
	// fix). Done off the startup path and gated on a successful read so it waits
	// for core + the servo-control broker to come up; EEPROM-backed gains persist,
	// SRAM acceleration is re-applied every boot.
	go applyServoTuning(m, sv, cfg.Tuning, order)

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
				// While teleop is actively commanding, don't read the servo bus:
				// the 50 Hz goal stream needs it, and the reads would be discarded
				// by the reseed gate anyway. Publish the loop's integrated estimate
				// so the panel still shows live angles without bus contention.
				if q, grip, active := loop.Estimate(); active {
					ja := jointAnglesFromEstimate(order, q, grip, cals)
					if b, err := proto.Marshal(ja); err == nil {
						_ = m.Publish(m.Subject("joints"), b)
					}
					continue
				}
				ja := jointstate.BuildJointAngles([]uint32{1, 2, 3, 4, 5, 6}, sv, cals)
				if b, err := proto.Marshal(ja); err == nil {
					_ = m.Publish(m.Subject("joints"), b)
				}
				loop.SetJointEstimate(jointEstimateFrom(ja))
				if g, ok := gripEstimateFrom(ja); ok {
					loop.SetGripEstimate(g)
				}
			}
		}
	}()

	// ~5 Hz servo-stats publisher feeds the Arm tab's MOTOR DETAIL card with
	// full per-servo telemetry (position, speed, load, current, voltage, temp).
	// Slower than the 30 Hz joints loop to keep bus contention down, and yields
	// to the calibration recorder for the same chain-starvation reason.
	go func() {
		t := time.NewTicker(200 * time.Millisecond)
		defer t.Stop()
		for {
			select {
			case <-m.Done():
				return
			case <-t.C:
				if ctrl.Recording() {
					continue
				}
				stats := jointstate.BuildServoStats([]uint32{1, 2, 3, 4, 5, 6}, sv)
				stats.T = timestamppb.Now()
				if b, err := proto.Marshal(stats); err == nil {
					_ = m.Publish(m.Subject("stats"), b)
				}
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

// gripEstimateFrom pulls the gripper (joint 6) angle from published JointAngles
// to seed the teleop loop's hold-to-move gripper. ok is false when joint 6 is
// N/A, leaving the prior estimate untouched.
func gripEstimateFrom(ja *so100v1.JointAngles) (float64, bool) {
	for _, j := range ja.GetJoints() {
		if j.GetId() == 6 && j.AngleRad != nil {
			return float64(j.GetAngleRad()), true
		}
	}
	return 0, false
}

// jointAnglesFromEstimate renders the teleop loop's integrated joint estimate
// (rad) as JointAngles for the panel during active teleop, so live angles keep
// updating without the servo-bus reads competing with the goal stream. q[0..4]
// are joints 1..5; grip is joint 6. A joint with no calibration is emitted N/A.
func jointAnglesFromEstimate(order []uint32, q [5]float64, grip float64, cals map[uint32]calibration.JointCal) *so100v1.JointAngles {
	ja := &so100v1.JointAngles{}
	for _, id := range order {
		j := &so100v1.Joint{Id: id}
		if _, ok := cals[id]; ok {
			rad := grip
			if id >= 1 && id <= 5 {
				rad = q[id-1]
			}
			j.AngleRad = proto.Float32(float32(rad))
		} else {
			j.NaReason = "uncalibrated"
		}
		ja.Joints = append(ja.Joints, j)
	}
	return ja
}

// applyServoTuning waits until the servo bus answers a read (core and the servo
// broker are up), then writes the one-time control-loop tuning to every joint.
// Best-effort and idempotent: EEPROM-backed gains persist across boots, SRAM
// acceleration is restored each boot. Gives up after a bounded wait.
func applyServoTuning(m *wpmodule.M, sv *servobus.Adapter, t config.ServoTuning, order []uint32) {
	tn := wpmodule.Tuning{
		PCoefficient:    proto.Uint32(t.PCoefficient),
		ICoefficient:    proto.Uint32(t.ICoefficient),
		DCoefficient:    proto.Uint32(t.DCoefficient),
		Acceleration:    proto.Uint32(t.Acceleration),
		MaxAcceleration: proto.Uint32(t.MaxAcceleration),
		ReturnDelay:     proto.Uint32(t.ReturnDelay),
	}
	for attempt := 0; attempt < 10; attempt++ {
		if _, err := sv.Read(order[0]); err == nil {
			for _, id := range order {
				if err := sv.SetTuning(id, tn); err != nil {
					slog.Warn("servo tuning write failed", "id", id, "err", err)
				}
			}
			slog.Info("servo control-loop tuning applied",
				"p", t.PCoefficient, "accel", t.Acceleration, "return_delay", t.ReturnDelay)
			return
		}
		select {
		case <-m.Done():
			return
		case <-time.After(500 * time.Millisecond):
		}
	}
	slog.Warn("servo tuning skipped: bus not reachable at startup")
}
