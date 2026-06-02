# SO-101 arm model (vendored)

These files are vendored from The Robot Studio's SO-ARM100 project. They are
copyright The Robot Studio and carry their own Apache-2.0 license, distinct
from Waypoint's own Apache-2.0 code.

- **Source:** https://github.com/TheRobotStudio/SO-ARM100 (`Simulation/SO101/`)
- **Copyright:** The Robot Studio and SO-ARM100 contributors
- **License:** Apache-2.0. Full text bundled alongside these files in [`LICENSE`](./LICENSE).
- **Modifications:** `so101.urdf` is upstream `so101_new_calib.urdf` renamed; the
  `assets/*.stl` meshes are unmodified.

Used here to render the arm in the dashboard. SO-101 is the current iteration of
the SO-100 arm family: identical 6-servo layout and joint names (servo IDs 1 to
6), near-identical geometry.
