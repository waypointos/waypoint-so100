# waypoint-module-so100

The SO-101 arm as a Waypoint no-rebuild module. Built on the Waypoint module
SDK (`github.com/waypoint-rover/waypoint/sdk`): the SDK owns connect, creds,
sd_notify, health, and stats; this repo provides the arm's own logic
(calibration, IK teleop, the joints publisher) and serves the standard
waypoint.v1 arm component API.

## Layout

- `cmd/waypoint-module-so100` — entrypoint on `wpmodule.Run`.
- `internal/servobus` — adapts the SDK servo client to the so100-internal
  interfaces (calibration, teleop sink, joint reader).
- `internal/armserver` — serves the standard `module.so100.arm.{state,cmd}` API
  over the calibrated joint path.
- `internal/{calibration,control,ik,teleop,jointstate,config}` — arm logic.

The standard arm API is a floor: the private surfaces (`module.so100.calibration`,
`module.so100.command`, `module.so100.joints`, the teleop window) remain.

## Build and test

```
go build ./...
go test ./...
```

The repo uses local `replace` directives pointing at a sibling `../waypoint`
checkout for the SDK and protocol bindings.

## Component classes and conformance

The standard arm surface is conformance-checked in the Waypoint repo's
`sim/conformance` suite via the in-tree `arm-sim` example (health, stats, state
stream, command-acts-through-broker, estop-freeze). so100's own correctness is
its unit tests plus the manual dev loop below; the contract it implements is the
same one `arm-sim` proves.

## Manual dev loop against a dev rover

In the Waypoint repo, boot a dev rover on the bench platform (six module-owned
servos):

```
make dev-rover PLATFORM=bench
```

In this repo, run the module against that rover with the dev env contract (dev
NATS is open, so no creds are minted):

```
WAYPOINT_NATS_URL=nats://127.0.0.1:4222 \
WAYPOINT_ROVER_ID=sim-rover \
WAYPOINT_MODULE_ID=so100 \
WAYPOINT_MODULE_COMPONENT=arm \
WAYPOINT_MODULE_STATE_RATE_HZ=30 \
go run ./cmd/waypoint-module-so100
```

Confirm `module.so100.arm.state` traffic on the rover's bus (`nats sub` on the
dev NATS, or the dashboard Bus pane). This is the hardware-day verification step
when a dev rover is available.
