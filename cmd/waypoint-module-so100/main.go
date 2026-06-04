package main

import (
	"bufio"
	"context"
	"flag"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/coreos/go-systemd/v22/daemon"
	natsgo "github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/waypoint-rover/waypoint-so100/internal/calibration"
	"github.com/waypoint-rover/waypoint-so100/internal/config"
	"github.com/waypoint-rover/waypoint-so100/internal/control"
	"github.com/waypoint-rover/waypoint-so100/internal/ik"
	"github.com/waypoint-rover/waypoint-so100/internal/jointstate"
	"github.com/waypoint-rover/waypoint-so100/internal/servo"
	"github.com/waypoint-rover/waypoint-so100/internal/teleop"
	so100v1 "github.com/waypoint-rover/waypoint-so100/protocol/gen/go"
)

func main() {
	configPath := flag.String("config", env("WAYPOINT_MODULE_CONFIG", ""), "path to config.toml")
	credsPath := flag.String("creds", env("WAYPOINT_MODULE_CREDS", ""), "path to creds.env")
	natsURL := flag.String("nats", env("WAYPOINT_NATS_URL", natsgo.DefaultURL), "nats URL")
	roverID := flag.String("rover", env("WAYPOINT_ROVER_ID", ""), "rover id")
	flag.Parse()
	slog.SetDefault(slog.New(slog.NewTextHandler(os.Stderr, nil)))
	if *credsPath == "" || *roverID == "" {
		slog.Error("waypoint-module-so100: --creds and --rover are required")
		os.Exit(2)
	}
	cfg, err := config.Load(*configPath)
	if err != nil {
		slog.Error(fmt.Sprintf("config: %v", err))
		os.Exit(1)
	}
	user, pass, err := loadCredsEnv(*credsPath)
	if err != nil {
		slog.Error(fmt.Sprintf("creds: %v", err))
		os.Exit(1)
	}
	ctx, cancel := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer cancel()
	nc, err := natsgo.Connect(*natsURL, natsgo.UserInfo(user, pass), natsgo.MaxReconnects(-1), natsgo.ReconnectWait(2*time.Second))
	if err != nil {
		slog.Error(fmt.Sprintf("nats connect: %v", err))
		os.Exit(1)
	}
	defer nc.Close()
	_, _ = daemon.SdNotify(false, daemon.SdNotifyReady)

	calSubject := fmt.Sprintf("waypoint.%s.module.so100.calibration", *roverID)
	publish := func(cals []calibration.JointCal, phase string, active uint32) {
		st := &so100v1.CalibrationState{T: timestamppb.Now(), Phase: phase, ActiveJoint: active}
		for _, c := range cals {
			jc := &so100v1.JointCalibration{
				Id: c.ID, Ok: c.OK,
				RawMin: proto.Uint32(uint32(c.RawMin)), RawMax: proto.Uint32(uint32(c.RawMax)),
				ZeroRaw: proto.Float64(c.ZeroRaw),
				SoftMin: proto.Uint32(uint32(c.SoftMin)), SoftMax: proto.Uint32(uint32(c.SoftMax)),
				MeasuredSpan: proto.Int32(int32(c.MeasuredSpan)), ExpectedSpan: proto.Int32(int32(c.ExpectedSpan)),
			}
			st.Joints = append(st.Joints, jc)
		}
		body, _ := proto.Marshal(st)
		_ = nc.Publish(calSubject, body)
		if phase == "done" {
			_ = calibration.Save(cfg.StatePath, cals)
		}
	}

	cl := servo.New(nc, *roverID)
	ctrl := control.New(cl, publish)

	// Health responder.
	_, _ = nc.Subscribe(fmt.Sprintf("waypoint.%s.module.so100.health.ready", *roverID), func(m *natsgo.Msg) {
		_ = m.Respond([]byte("ok"))
	})
	// Command handler: run / abort calibration. Calibration runs in a goroutine
	// so the subscription callback returns promptly.
	_, _ = nc.Subscribe(fmt.Sprintf("waypoint.%s.module.so100.command", *roverID), func(m *natsgo.Msg) {
		var cmd so100v1.ArmCommand
		if proto.Unmarshal(m.Data, &cmd) != nil {
			return
		}
		if cmd.GetRunCalibration() {
			go ctrl.RunCalibration()
		}
	})

	// Publish the persisted calibration once at boot so the tab shows last state.
	cals := map[uint32]calibration.JointCal{}
	if loaded, err := calibration.Load(cfg.StatePath); err == nil {
		publish(loaded, "idle", 0)
		for _, c := range loaded {
			cals[c.ID] = c
		}
	}

	// Teleop: subscribe gamepad input, run the 50Hz IK control loop, and poll
	// calibrated joint angles for the render window.
	loop := teleop.NewLoop(teleop.LoopConfig{
		StaleAfter: 150 * time.Millisecond,
		MaxLinear:  0.15, MaxPitch: 1.0, RampPerTick: 0.02, Dt: 0.02,
	}, cl, cals, ik.SO100Kinematics())

	_, _ = nc.Subscribe(fmt.Sprintf("waypoint.%s.module.so100.input", *roverID), func(m *natsgo.Msg) {
		var s so100v1.GamepadSnapshot
		if proto.Unmarshal(m.Data, &s) != nil {
			return
		}
		loop.SetInput(&s, time.Now())
	})

	stop := make(chan struct{})
	go loop.Run(stop)

	go func() {
		t := time.NewTicker(33 * time.Millisecond)
		defer t.Stop()
		for {
			select {
			case <-stop:
				return
			case <-t.C:
				ja := jointstate.BuildJointAngles([]uint32{1, 2, 3, 4, 5, 6}, cl, cals)
				if b, err := proto.Marshal(ja); err == nil {
					_ = nc.Publish(jointstate.PublishSubject(*roverID, "so100"), b)
				}
				loop.SetJointEstimate(jointEstimateFrom(ja))
			}
		}
	}()

	<-ctx.Done()
	close(stop)
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

func env(key, def string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return def
}

func loadCredsEnv(path string) (string, string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", "", err
	}
	defer f.Close()
	var user, pass string
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		k, v, ok := strings.Cut(line, "=")
		if !ok {
			continue
		}
		switch k {
		case "WAYPOINT_NATS_USER":
			user = v
		case "WAYPOINT_NATS_PASSWORD":
			pass = v
		}
	}
	if user == "" || pass == "" {
		return "", "", fmt.Errorf("creds: missing user or password")
	}
	return user, pass, nil
}
