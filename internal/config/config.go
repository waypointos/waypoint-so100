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
}

func Load(path string) (*Config, error) {
	cfg := &Config{
		StatePath:     "/var/lib/waypoint-module-so100/calibration.toml",
		PosesPath:     "/var/lib/waypoint-module-so100/poses.toml",
		ShareButton:   8,
		OptionsButton: 9,
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
	return cfg, nil
}
