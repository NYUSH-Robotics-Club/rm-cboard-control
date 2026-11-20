# Quest Trajectory Recorder

An input-device adapter for the controller-tracking Quest APK. Its maintained
runtime boundary is deliberately small:

```text
Quest APK raw ports -> parser -> named calibration -> TeleopTarget ZMQ PUB
```

## Persistent calibration editor

The source is the only raw-input owner. While `just source <profile>` is running,
its editor remains available at `http://127.0.0.1:8766/`. `just calibrate <profile>`
reuses this editor when present, without starting a second receiver. Without a
running source it starts the standalone, page-first editor as before.

1. **Start new calibration** closes the published gate and sets
   `source_metadata.calibration_valid=false`. Only the editor receives live poses.
2. Collect right, finish; collect forward, finish; then set origin.
3. **Finish Calibration** validates and atomically saves the named profile, then
   applies it to the running source. The page stays open but stops pose updates.
4. Pause B, then resume B. New targets use the saved transform. **Cancel** retains
   the previous profile and also requires B before resuming.

Every button reports progress and a persistent result in the editor. Finish
shows **Saving…**, then **Saved ✓** only after acknowledgement. Failed requests
show the reason and retain the draft; unconfirmed requests time out after four
seconds. Profile-list and pose refreshes cannot overwrite action feedback.

The source keeps publishing status and gated targets throughout; subscribers
must honor the gate and calibration validity. Revisions change at mode boundaries
so consumers can re-anchor even if they miss the invalidation packet. Metadata
includes `calibration_editor`, `calibration_revision`, `effective_calibration`, and
the actual saved file's `calibration_sha256`. There is no separate Align workflow
or automatic invalidation based on ADB log availability. The recovered APK cannot
guarantee persistent coordinates across recentering/relocalization: recalibrate
in this same editor when directions are wrong. Reconnect never restarts the APK.

Source configuration uses local ZMQ REP `tcp://127.0.0.1:8133`
(`--source-control-bind`) or `POST /editor/command`. Requests contain `request_id`
and `action` (`status`, `begin`, `finish`, `cancel`). Finish/cancel also require
the current `revision`; finish includes `profile` and the complete `calibration`.
Responses include `accepted`, `applied`, `message`, and `editor`. Begin is
idempotent; old-session saves are rejected; repeated IDs replay their result.
`GET /editor/status` is read-only. Keep both endpoints on a trusted host; they
are not authenticated. `--web-host` and `--web-port` configure the editor.
`just status-json` reports a running source-owned editor separately from raw
tracking availability. Stopping the standalone editor never stops a live source.

This repository does not know which simulator, robot, recorder, or UI consumes
the stream. It does not subscribe to backend feedback, send robot commands, or
supervise downstream processes. The canonical message and ZMQ transport come
from the versioned `embodied-ops[teleop-zmq]` package.

## Daily workflow

Install once:

```bash
scripts/setup.sh
```

Prepare the headset and calibrate when the physical setup changes:

```bash
just adb-prepare
just calibrate lab_new
just status-json
```

Named profiles are stored outside the checkout at:

```text
${QUEST_CALIBRATION_DIR:-~/.config/quest-trajectory-recorder/calibrations}
```

Legacy `calibrations/<name>.json` files remain readable and can be copied with
`quest-profile migrate <name>`. Inspect resolution with `just profiles` and
`just profile-path <name>`.

Publish a calibrated target stream directly:

```bash
just source lab_new
# equivalent installed command:
quest-source --profile lab_new --target-bind tcp://127.0.0.1:8130
```

`quest-source` waits for authorized ADB, wakes Quest, restores reverse ports,
soft-focuses FrankaBot without restarting it, validates the profile, and then
executes only the target publisher. Stop it with Ctrl+C or let a downstream
repository own it as one child of that repository's session.

For complete collection, run the session command exposed by the backend that
owns the robot. That package may launch `quest-source` as an opaque child, but
this package neither discovers nor calls it. The backend owns native mapping,
cameras, recording, visualization, full-session status, and cleanup.

## Quest application

- package: `com.Xigbee.FrankaBot`
- activity/build: `FrankaBotControllerTracking`
- controller pose: raw port 8125
- gripper events: raw port 8127
- resolution state: raw port 8095
- B-button stream gate: raw port 8100
- canonical target/status PUB: default port 8130
- calibration page: HTTP 8766

The observed APK emits the right controller and host receive timing only; it
does not provide a device-side sample timestamp. The publisher preserves raw
metadata, source/session/frame identity, calibration SHA-256, tracking validity,
gate state, pose, orientation, and gripper command in
`embodied.teleop_target/v1` / `embodied.teleop_source_status/v1`.

## ADB operations

`just adb-prepare` is safe to retry: wait for ADB, wake Quest, enable stay-awake,
repair reverse mappings, close launch-blocking Meta panels, focus FrankaBot, and
verify XR foreground. `just adb-focus` never restarts the app;
`just adb-restart` is the explicit last resort.

If the headset is awake but poses become exact zero, first suspect Meta system
UI focus. If the controller is `CONNECTED_INACTIVE`, wear the headset and wave
the controller in camera view; ADB health alone cannot restore 6DoF tracking.

## Development

```bash
just check
```

See [architecture](docs/architecture.md) for the strict ownership rules and
[`scripts/README.md`](scripts/README.md) for low-level diagnostic entry points.
