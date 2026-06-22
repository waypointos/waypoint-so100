// Package poses stores named arm poses and recalls them on command. A pose is
// the raw encoder position of every servo captured while the arm was posed by
// hand (torque off). Recall writes those raw goals straight back, bypassing IK
// and the rad<->raw calibration round-trip, so replay is exact and survives
// calibration drift. It mirrors internal/calibration's TOML store pattern.
package poses

import (
	"errors"
	"os"
	"path/filepath"
	"sort"
	"sync"

	"github.com/BurntSushi/toml"
)

// Slots are the two recallable bindings, surfaced as the gamepad Share / Options
// buttons and the on-screen teleop buttons.
const (
	SlotShare   = "share"
	SlotOptions = "options"
)

// Slots is the canonical slot order for publishing/rendering.
var Slots = []string{SlotShare, SlotOptions}

// ServoPos is one servo's captured raw encoder position.
type ServoPos struct {
	ID  uint32 `toml:"id"`
	Raw uint16 `toml:"raw"`
}

// Pose is the full captured pose for a slot.
type Pose struct {
	Slot  string     `toml:"slot"`
	Name  string     `toml:"name"`
	Servo []ServoPos `toml:"servo"`
}

// RawReader reads a servo's present raw position (satisfied by servobus.Adapter).
type RawReader interface {
	ReadRaw(id uint32) (uint16, bool)
}

type fileFormat struct {
	Pose []Pose `toml:"pose"`
}

// Save writes the pose set atomically to path, creating parent dirs.
func Save(path string, poses []Pose) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	tmp := path + ".tmp"
	f, err := os.Create(tmp)
	if err != nil {
		return err
	}
	if err := toml.NewEncoder(f).Encode(fileFormat{Pose: poses}); err != nil {
		f.Close()
		return err
	}
	if err := f.Close(); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}

// Load reads the pose set; a missing file yields an empty slice.
func Load(path string) ([]Pose, error) {
	data, err := os.ReadFile(path)
	if errors.Is(err, os.ErrNotExist) {
		return nil, nil
	}
	if err != nil {
		return nil, err
	}
	var ff fileFormat
	if _, err := toml.Decode(string(data), &ff); err != nil {
		return nil, err
	}
	sort.Slice(ff.Pose, func(i, j int) bool { return ff.Pose[i].Slot < ff.Pose[j].Slot })
	return ff.Pose, nil
}

// Store is the in-memory pose set, keyed by slot. Safe for concurrent use: the
// command handler mutates it while the teleop loop reads a snapshot via Map.
type Store struct {
	mu     sync.Mutex
	byID   map[string]Pose
	reader RawReader
}

// NewStore builds a store seeded with loaded poses, reading live positions
// through reader on Capture.
func NewStore(reader RawReader, loaded []Pose) *Store {
	s := &Store{byID: map[string]Pose{}, reader: reader}
	for _, p := range loaded {
		s.byID[p.Slot] = p
	}
	return s
}

// Capture reads the present raw position of every id and binds it to slot under
// name, replacing any existing pose there. It returns an error only if no servo
// could be read (an all-or-nothing read guards against saving a half-empty pose).
func (s *Store) Capture(slot, name string, ids []uint32) (Pose, error) {
	servos := make([]ServoPos, 0, len(ids))
	for _, id := range ids {
		if raw, ok := s.reader.ReadRaw(id); ok {
			servos = append(servos, ServoPos{ID: id, Raw: raw})
		}
	}
	if len(servos) == 0 {
		return Pose{}, errors.New("poses: no servo could be read")
	}
	p := Pose{Slot: slot, Name: name, Servo: servos}
	s.mu.Lock()
	s.byID[slot] = p
	s.mu.Unlock()
	return p, nil
}

// Set inserts or replaces a pose, used to seed the store from disk at boot.
func (s *Store) Set(p Pose) {
	s.mu.Lock()
	s.byID[p.Slot] = p
	s.mu.Unlock()
}

// Delete clears a slot.
func (s *Store) Delete(slot string) {
	s.mu.Lock()
	delete(s.byID, slot)
	s.mu.Unlock()
}

// Get returns the pose bound to slot.
func (s *Store) Get(slot string) (Pose, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	p, ok := s.byID[slot]
	return p, ok
}

// All returns the stored poses, slot-sorted, for persistence.
func (s *Store) All() []Pose {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]Pose, 0, len(s.byID))
	for _, p := range s.byID {
		out = append(out, p)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Slot < out[j].Slot })
	return out
}

// Map returns a slot -> (servo id -> raw) snapshot for the teleop loop's recall.
func (s *Store) Map() map[string]map[uint32]uint16 {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make(map[string]map[uint32]uint16, len(s.byID))
	for slot, p := range s.byID {
		m := make(map[uint32]uint16, len(p.Servo))
		for _, sp := range p.Servo {
			m[sp.ID] = sp.Raw
		}
		out[slot] = m
	}
	return out
}
