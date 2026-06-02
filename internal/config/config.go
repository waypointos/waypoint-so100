package config

import (
	"os"

	"github.com/BurntSushi/toml"
)

type Config struct {
	StatePath string // default /var/lib/waypoint-module-so100/calibration.toml
}

func Load(path string) (*Config, error) {
	cfg := &Config{StatePath: "/var/lib/waypoint-module-so100/calibration.toml"}
	data, err := os.ReadFile(path)
	if err != nil {
		return cfg, nil // config is optional; defaults are fine
	}
	var raw struct {
		StatePath string `toml:"state_path"`
	}
	if _, err := toml.Decode(string(data), &raw); err != nil {
		return nil, err
	}
	if raw.StatePath != "" {
		cfg.StatePath = raw.StatePath
	}
	return cfg, nil
}
