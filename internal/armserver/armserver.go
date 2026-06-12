// Package armserver adapts so100's calibrated joint state and goal path to the
// standard waypoint.v1 arm component API. It is a floor over the existing
// servo path: the private calibration, joints, and teleop surfaces are unchanged.
package armserver

import (
	"sync"

	waypointv1 "github.com/waypoint-rover/waypoint/protocol/gen/go/messages"

	"github.com/waypoint-rover/waypoint-so100/internal/calibration"
)

// RawReader returns a servo's present raw position and whether the read succeeded.
type RawReader interface {
	ReadRaw(id uint32) (uint16, bool)
}

// GoalWriter writes calibrated raw goals through the servo-control broker.
type GoalWriter interface {
	SyncWriteGoals(goals []*waypointv1.ServoGoal) error
	SetTorqueEnable(id uint32, on bool) error
}

type Server struct {
	mu     sync.Mutex
	reader RawReader
	writer GoalWriter
	cals   map[uint32]calibration.JointCal
	names  map[uint32]string // bus id -> joint name, config-driven
	order  []uint32          // state order
	halt   func()            // freezes the teleop loop on ArmCommand stop
}

func New(reader RawReader, writer GoalWriter, cals map[uint32]calibration.JointCal, names map[uint32]string, order []uint32) *Server {
	return &Server{reader: reader, writer: writer, cals: cals, names: names, order: order}
}

// SetHalt wires the motion-halt hook (the teleop loop's Halt) invoked on stop.
func (s *Server) SetHalt(fn func()) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.halt = fn
}

// SetCalibration swaps the calibration map after a calibration run.
func (s *Server) SetCalibration(cals map[uint32]calibration.JointCal) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.cals = cals
}

func (s *Server) State() *waypointv1.ArmState {
	s.mu.Lock()
	defer s.mu.Unlock()
	st := &waypointv1.ArmState{}
	for _, id := range s.order {
		j := &waypointv1.ArmJoint{Name: s.names[id]}
		raw, ok := s.reader.ReadRaw(id)
		cal, hasCal := s.cals[id]
		if ok && hasCal && cal.OK {
			j.PositionRad = cal.ThetaRad(raw)
			j.Calibrated = true
		}
		st.Joints = append(st.Joints, j)
	}
	return st
}

func (s *Server) Command(cmd *waypointv1.ArmCommand) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if cmd.GetStop() {
		// Freeze the teleop loop, then re-latch every readable joint's goal to
		// its present position so any in-flight move stops where it is.
		if s.halt != nil {
			s.halt()
		}
		var hold []*waypointv1.ServoGoal
		for _, id := range s.order {
			if raw, ok := s.reader.ReadRaw(id); ok {
				hold = append(hold, &waypointv1.ServoGoal{ServoId: id, GoalPosition: uint32(raw)})
			}
		}
		if len(hold) == 0 {
			return nil
		}
		return s.writer.SyncWriteGoals(hold)
	}
	g := cmd.GetGoals()
	if g == nil {
		return nil
	}
	var goals []*waypointv1.ServoGoal
	for _, goal := range g.GetGoals() {
		for id, name := range s.names {
			if name != goal.GetName() {
				continue
			}
			cal, ok := s.cals[id]
			if !ok || !cal.OK {
				continue // uncalibrated joints refuse standard goals
			}
			raw := cal.RawFromRad(goal.GetPositionRad())
			_ = s.writer.SetTorqueEnable(id, true)
			goals = append(goals, &waypointv1.ServoGoal{ServoId: id, GoalPosition: uint32(raw)})
		}
	}
	if len(goals) == 0 {
		return nil
	}
	return s.writer.SyncWriteGoals(goals)
}
