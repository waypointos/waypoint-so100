package config

import (
	"os"

	"github.com/BurntSushi/toml"
)

type Config struct {
	StatePath string // default /var/lib/waypoint-module-so100/calibration.toml
	PosesPath string // default /var/lib/waypoint-module-so100/poses.toml
	// ShareButton / OptionsButton are the gamepad buttons[] indices that recall
	// the "share" / "options" pose slots. The SDK delivers buttons[] and
	// triggers[] split apart (not the raw W3C layout), so the true index is
	// hardware-dependent: the teleop loop logs every pressed index at Debug so
	// these can be confirmed and overridden here.
	ShareButton   int
	OptionsButton int

	// Tuning is the servo control-loop tuning applied once at startup (torque
	// off). Lowering the position-loop P gain and raising acceleration is what
	// lets streamed goal positions glide instead of dart-and-stall — the teleop
	// jitter fix. Defaults follow LeRobot's SO-arm recipe; override per rover.
	Tuning ServoTuning
}

// ServoTuning mirrors the STS3215 control-loop registers written at startup.
type ServoTuning struct {
	PCoefficient    uint32 // position-loop P (reg 0x15); lower = less shake (firmware default 32)
	ICoefficient    uint32 // position-loop I (reg 0x17)
	DCoefficient    uint32 // position-loop D (reg 0x16)
	Acceleration    uint32 // reg 0x29, goal accel/decel ramp
	MaxAcceleration uint32 // reg 0x55, accel ceiling
	ReturnDelay     uint32 // reg 0x07, bus response delay; 0 = minimum (~2µs)
}

func Load(path string) (*Config, error) {
	cfg := &Config{
		StatePath:     "/var/lib/waypoint-module-so100/calibration.toml",
		PosesPath:     "/var/lib/waypoint-module-so100/poses.toml",
		ShareButton:   8,
		OptionsButton: 9,
		Tuning: ServoTuning{
			PCoefficient: 16, ICoefficient: 0, DCoefficient: 32,
			Acceleration: 254, MaxAcceleration: 254, ReturnDelay: 0,
		},
	}
	data, err := os.ReadFile(path)
	if err != nil {
		return cfg, nil // config is optional; defaults are fine
	}
	var raw struct {
		StatePath     string `toml:"state_path"`
		PosesPath     string `toml:"poses_path"`
		ShareButton   *int   `toml:"share_button"`
		OptionsButton *int   `toml:"options_button"`
		Tuning        *struct {
			PCoefficient    *uint32 `toml:"p_coefficient"`
			ICoefficient    *uint32 `toml:"i_coefficient"`
			DCoefficient    *uint32 `toml:"d_coefficient"`
			Acceleration    *uint32 `toml:"acceleration"`
			MaxAcceleration *uint32 `toml:"max_acceleration"`
			ReturnDelay     *uint32 `toml:"return_delay"`
		} `toml:"servo_tuning"`
	}
	if _, err := toml.Decode(string(data), &raw); err != nil {
		return nil, err
	}
	if raw.StatePath != "" {
		cfg.StatePath = raw.StatePath
	}
	if raw.PosesPath != "" {
		cfg.PosesPath = raw.PosesPath
	}
	if raw.ShareButton != nil {
		cfg.ShareButton = *raw.ShareButton
	}
	if raw.OptionsButton != nil {
		cfg.OptionsButton = *raw.OptionsButton
	}
	if t := raw.Tuning; t != nil {
		if t.PCoefficient != nil {
			cfg.Tuning.PCoefficient = *t.PCoefficient
		}
		if t.ICoefficient != nil {
			cfg.Tuning.ICoefficient = *t.ICoefficient
		}
		if t.DCoefficient != nil {
			cfg.Tuning.DCoefficient = *t.DCoefficient
		}
		if t.Acceleration != nil {
			cfg.Tuning.Acceleration = *t.Acceleration
		}
		if t.MaxAcceleration != nil {
			cfg.Tuning.MaxAcceleration = *t.MaxAcceleration
		}
		if t.ReturnDelay != nil {
			cfg.Tuning.ReturnDelay = *t.ReturnDelay
		}
	}
	return cfg, nil
}
