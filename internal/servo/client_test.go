package servo

import (
	"testing"
	"time"

	natsgo "github.com/nats-io/nats.go"
	natssrv "github.com/nats-io/nats-server/v2/server"
	nattest "github.com/nats-io/nats-server/v2/test"
	"github.com/stretchr/testify/require"
	"google.golang.org/protobuf/proto"

	so100v1 "github.com/waypoint-rover/waypoint-so100/protocol/gen/go"
)

func bus(t *testing.T) *natsgo.Conn {
	t.Helper()
	srv := nattest.RunServer(&natssrv.Options{Port: -1})
	t.Cleanup(srv.Shutdown)
	nc, err := natsgo.Connect(srv.ClientURL())
	require.NoError(t, err)
	t.Cleanup(nc.Close)
	return nc
}

func TestClient_SetGoalPositionPublishesServoControl(t *testing.T) {
	nc := bus(t)
	got := make(chan uint32, 1)
	_, err := nc.Subscribe("waypoint.rov.module.so100.servo.cmd", func(m *natsgo.Msg) {
		var c so100v1.ServoControl
		if proto.Unmarshal(m.Data, &c) == nil {
			got <- c.GetSetGoalPosition()
		}
	})
	require.NoError(t, err)

	cl := New(nc, "rov")
	require.NoError(t, cl.SetGoalPosition(3, 2048))
	select {
	case v := <-got:
		require.Equal(t, uint32(2048), v)
	case <-time.After(time.Second):
		t.Fatal("no ServoControl published")
	}
}

func TestClient_SyncWriteGoals_PublishesToServoSyncSubject(t *testing.T) {
	nc := bus(t)
	got := make(chan []uint32, 1)
	_, err := nc.Subscribe("waypoint.rov.module.so100.servo.sync", func(m *natsgo.Msg) {
		var s so100v1.ServoSyncWrite
		if proto.Unmarshal(m.Data, &s) == nil {
			ids := []uint32{}
			for _, g := range s.GetGoals() {
				ids = append(ids, g.GetServoId())
			}
			got <- ids
		}
	})
	require.NoError(t, err)

	cl := New(nc, "rov")
	require.NoError(t, cl.SyncWriteGoals([]*so100v1.ServoGoal{
		{ServoId: 1, GoalPosition: 2048}, {ServoId: 2, GoalPosition: 1000},
	}))
	select {
	case ids := <-got:
		require.Equal(t, []uint32{1, 2}, ids)
	case <-time.After(2 * time.Second):
		t.Fatal("sync write not published")
	}
}

func TestClient_ReadRequestsBroker(t *testing.T) {
	nc := bus(t)
	_, err := nc.Subscribe("waypoint.rov.module.so100.servo.read", func(m *natsgo.Msg) {
		st := &so100v1.ServoState{ServoId: 3, Ok: true, PositionRaw: proto.Uint32(1234), CurrentRaw: proto.Uint32(40)}
		b, _ := proto.Marshal(st)
		_ = m.Respond(b)
	})
	require.NoError(t, err)

	cl := New(nc, "rov")
	r, err := cl.Read(3)
	require.NoError(t, err)
	require.True(t, r.OK)
	require.Equal(t, uint16(1234), r.PositionRaw)
}
