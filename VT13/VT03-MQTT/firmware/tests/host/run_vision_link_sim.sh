#!/usr/bin/env sh
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
test_build=$(mktemp -d "${TMPDIR:-/tmp}/vision-link-sim.XXXXXX")
trap 'rm -rf "$test_build"' EXIT INT TERM
cc=${CC:-gcc}

"$cc" -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections \
  -I"$repo_root/application/cmd" -I"$repo_root/adapters/vision" \
  -I"$repo_root/config" -I"$repo_root/core/chassis" \
  -I"$repo_root/core/common" -I"$repo_root/core/contracts" \
  -I"$repo_root/modules/message_center" -I"$repo_root/modules/vision_comm" \
  -I"$repo_root/modules/logger" -I"$repo_root/modules/imu" \
  -I"$repo_root/bsp/time" -I"$repo_root/bsp/usb" -I"$repo_root/bsp/critical" \
  "$repo_root/bsp/critical/bsp_critical.c" \
  "$repo_root/modules/message_center/message_center.c" \
  "$repo_root/modules/vision_comm/vision_comm.c" \
  "$repo_root/modules/vision_comm/seasky_protocol.c" \
  "$repo_root/modules/vision_comm/crc8.c" \
  "$repo_root/modules/vision_comm/crc16.c" \
  "$repo_root/adapters/vision/legacy_vision_bridge.c" \
  "$repo_root/application/cmd/command_router.c" \
  "$repo_root/tests/host/test_vision_link_sim.c" \
  -Wl,--gc-sections -lm -o "$test_build/vision_link_sim"
"$test_build/vision_link_sim"
