package calibration

import (
	"errors"
	"os"
	"path/filepath"
	"sort"

	"github.com/BurntSushi/toml"
)

type fileFormat struct {
	Joint []JointCal `toml:"joint"`
}

// Save writes the calibration set atomically to path, creating parent dirs.
func Save(path string, cals []JointCal) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	tmp := path + ".tmp"
	f, err := os.Create(tmp)
	if err != nil {
		return err
	}
	if err := toml.NewEncoder(f).Encode(fileFormat{Joint: cals}); err != nil {
		f.Close()
		return err
	}
	if err := f.Close(); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}

// Load reads the calibration set; a missing file yields an empty slice.
func Load(path string) ([]JointCal, error) {
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
	sort.Slice(ff.Joint, func(i, j int) bool { return ff.Joint[i].ID < ff.Joint[j].ID })
	return ff.Joint, nil
}
