# Sentry Bridge

This repository now accepts the reference sentry bridge SX command stream on
the STM32 USB CDC link and returns ST telemetry on the same link.

## Data flow

```text
ROS2 /cmd_vel_chassis or radar node
  -> radar PTY (legacy radar frames)
  -> scripts/sentry_bridge.py
  -> USB CDC
  -> SX stream parser
  -> TOPIC_CHASSIS_CMD
  -> sentry chassis controller
  -> existing swerve kinematics and CAN motor service
```

The bridge owns the real STM32 serial device. It creates a vision PTY and a
radar PTY, and uses a per-device lock file so a second bridge cannot open the
same device.

## Start

On the Linux computer connected to the board:

```sh
python3 -m pip install -r scripts/requirement.txt
python3 scripts/sentry_bridge.py --port /dev/ttyACM0
```

The default links are `/tmp/nyush-rm-sentry-vision` and
`/tmp/nyush-rm-sentry-radar`. Set `SENTRY_BRIDGE_VX_SIGN` or
`SENTRY_BRIDGE_VY_SIGN` to `-1` only after the installed robot axes have been
validated.

## Wire protocol

SX is 33 bytes:

```text
'S','X', vx, vy, wz, gimbal_yaw_delta, gimbal_pitch_delta,
control_flags, scan_yaw_rate_deg_s, search_pitch_deg, crc16
```

The five values before `control_flags` and the two values after it are little
endian IEEE-754 float32. The final two bytes are little endian CRC16/X25 over
the preceding 31 bytes. `search_pitch_deg` may be NaN.

ST is 39 bytes:

```text
'S','T', cmd_vx, cmd_vy, cmd_wz, real_vx, real_vy, real_wz,
robot_status, game_status, stage_remain_time, robot_id, current_hp,
shooter_heat, team_color, is_attacked, crc16
```

The current repository has no referee-data adapter or chassis velocity
estimator in this control path, so referee status and `real_*` are zero. The
wire format can carry measured velocity when an estimator is added later.

## Odin/Nav2 four-omni-wheel mode

For `infantry_standard` Odin navigation, SX `vx` and `vy` are normalized
translation in the **gimbal-forward** Nav2 frame, not SI velocities or direct
chassis axes. The Jetson sender divides Nav2 m/s by the configured translation
limit before building the legacy radar frame. The C board uses the same fresh
yaw-encoder conversion as middle-position RC follow mode to rotate this vector
into chassis axes in every RC switch position. Odin's rear-facing 180-degree
mount correction is already in Nav2's TF and must not be applied again. SX
`wz` remains a normalized chassis rotation request. The omni strategy then
multiplies the normalized values by the configured speed limits and computes
M3508 rotor RPM from wheel geometry. The wheel radius in this source is
0.0775 m for a measured 15.5 cm diameter; motor IDs, wheel positions,
reduction ratio and signs still need confirmation on this robot before flashing.

Only the Jetson bridge opens the physical USB CDC device. The ROS 2 sender
opens `/tmp/nyush-rm-sentry-radar`, sends 19-byte A5 5A radar velocity frames,
and stops sending nonzero commands when `/cmd_vel_chassis` is stale. The Odin
Nav2 launch starts this sender only in navigation mode. The chassis output
default is true in both the Odin navigation and Foxglove launches; pass
`enable_chassis_output:=false` to disable it.

On the omni firmware, all RC switch positions allow fresh Nav2 commands while
the chassis translation sticks and rotation dial are neutral. Moving any of
those inputs selects the RC chassis command immediately. Gimbal stick motion
does not interrupt Nav2 chassis control. RC loss, CAN disarm, invalid normalized
SX values, SX expiry, or missing/stale yaw-encoder feedback exclude Nav2.
Invalid feedback zeros the autonomous chassis command, including in the RC
spin position; a nonneutral RC chassis input still takes manual priority.
This policy does not change the existing sentry-swerve bridge override.

The `ST.real_*` fields are still not measured velocity. Nav2 must continue to
use Odin localization and `/odin1/odometry` rather than bridge odometry.

## Safety behavior

The STM32 accepts only CRC-valid SX frames. The latest valid frame expires
after 250 ms. An expired or missing frame is excluded from the chassis command
and the existing command controller publishes its normal disabled/RC result.
The bridge also emits zero SX frames after radar timeout and during shutdown.

The bridge's radar parser handles partial frames, concatenated frames, and
bad CRC by resynchronizing at the next header. Vision traffic remains raw
pass-through traffic on the vision PTY.
