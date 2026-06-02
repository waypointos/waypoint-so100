// Package servo is the so100 module's client to the agent's servo-control
// broker. It publishes ServoControl to module.so100.servo.cmd and requests
// module.so100.servo.read; the agent bridges both to core.
package servo

import (
	"fmt"
	"time"

	natsgo "github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"

	"github.com/waypoint-rover/waypoint-so100/internal/calibration"
	so100v1 "github.com/waypoint-rover/waypoint-so100/protocol/gen/go"
)

type Client struct {
	nc      *natsgo.Conn
	cmdSubj string
	readSub string
}

func New(nc *natsgo.Conn, roverID string) *Client {
	return &Client{
		nc:      nc,
		cmdSubj: fmt.Sprintf("waypoint.%s.module.so100.servo.cmd", roverID),
		readSub: fmt.Sprintf("waypoint.%s.module.so100.servo.read", roverID),
	}
}

func (c *Client) pub(ctrl *so100v1.ServoControl) error {
	body, err := proto.Marshal(ctrl)
	if err != nil {
		return err
	}
	return c.nc.Publish(c.cmdSubj, body)
}

func (c *Client) SetMode(id uint32, mode uint32) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetMode{SetMode: mode}})
}
func (c *Client) SetTorqueLimit(id uint32, raw uint16) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetTorqueLimit{SetTorqueLimit: uint32(raw)}})
}
func (c *Client) SetOvercurrentLimit(id uint32, raw uint16) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetOvercurrentLimit{SetOvercurrentLimit: uint32(raw)}})
}
func (c *Client) SetAngleLimits(id uint32, min, max uint16) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetAngleLimits{
		SetAngleLimits: &so100v1.AngleLimits{MinRaw: uint32(min), MaxRaw: uint32(max)}}})
}
func (c *Client) EnableTorque(id uint32) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetTorqueEnable{SetTorqueEnable: true}})
}
func (c *Client) DisableTorque(id uint32) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetTorqueEnable{SetTorqueEnable: false}})
}
func (c *Client) SetGoalPosition(id uint32, raw uint16) error {
	return c.pub(&so100v1.ServoControl{ServoId: id, Op: &so100v1.ServoControl_SetGoalPosition{SetGoalPosition: uint32(raw)}})
}

func (c *Client) Read(id uint32) (calibration.ServoReading, error) {
	req, _ := proto.Marshal(&so100v1.ServoReadRequest{ServoId: id})
	msg, err := c.nc.Request(c.readSub, req, 2*time.Second)
	if err != nil {
		return calibration.ServoReading{}, err
	}
	var st so100v1.ServoState
	if err := proto.Unmarshal(msg.Data, &st); err != nil {
		return calibration.ServoReading{}, err
	}
	return calibration.ServoReading{
		PositionRaw: uint16(st.GetPositionRaw()),
		CurrentRaw:  uint16(st.GetCurrentRaw()),
		OK:          st.GetOk(),
	}, nil
}
