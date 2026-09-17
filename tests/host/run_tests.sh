#!/usr/bin/env sh
# Compile and run hardware-independent checks with the host C compiler.
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
test_build=$(mktemp -d "${TMPDIR:-/tmp}/rm-control-tests.XXXXXX")
trap 'rm -rf "$test_build"' EXIT INT TERM

cc=${CC:-clang}
common_flags="-std=c11 -Wall -Wextra -Werror"

"$cc" $common_flags -I"$repo_root/tests/host/stubs/can_recovery" -I"$repo_root/bsp/can" \
  "$repo_root/bsp/can/bsp_can.c" "$repo_root/tests/host/test_can_recovery.c" \
  -o "$test_build/test_can_recovery"
"$test_build/test_can_recovery"

"$cc" $common_flags -I"$repo_root/tests/host/stubs/alarm" -I"$repo_root/bsp/alarm" \
  "$repo_root/bsp/alarm/bsp_alarm.c" "$repo_root/tests/host/test_bsp_alarm.c" \
  -o "$test_build/test_bsp_alarm"
"$test_build/test_bsp_alarm"

"$cc" $common_flags -DROBOT_TYPE_infantry_standard \
  -I"$repo_root/config" -I"$repo_root/config/robots" \
  -I"$repo_root/core/common" -I"$repo_root/core/contracts" -I"$repo_root/core/motor" \
  -I"$repo_root/services/motor" -I"$repo_root/bsp/alarm" -I"$repo_root/application/diagnostics" \
  "$repo_root/application/diagnostics/motor_offline_alarm.c" \
  "$repo_root/tests/host/test_motor_offline_alarm.c" -o "$test_build/test_motor_offline_alarm"
"$test_build/test_motor_offline_alarm"

"$cc" $common_flags -std=gnu11 -DROBOT_TYPE_infantry_standard \
  -I"$repo_root/tests/host/stubs/can" -I"$repo_root/config" -I"$repo_root/config/robots" \
  -I"$repo_root/application/gimbal" -I"$repo_root/application/shoot" \
  -I"$repo_root/core/common" -I"$repo_root/core/contracts" -I"$repo_root/core/motor" \
  -I"$repo_root/bsp/can" -I"$repo_root/bsp/time" -I"$repo_root/bsp/critical" \
  -I"$repo_root/modules/motor" -I"$repo_root/modules/algorithm" -I"$repo_root/modules/can_comm" \
  -I"$repo_root/modules/debug_print" -I"$repo_root/modules/remote" -I"$repo_root/modules/logger" \
  -I"$repo_root/modules/message_center" -I"$repo_root/services/motor" \
  -I"$repo_root/application/dashboard" "$repo_root/tests/host/stubs/dashboard.c" \
  "$repo_root/application/gimbal/gimbal_controller.c" "$repo_root/application/gimbal/yaw_reference.c" \
  "$repo_root/application/shoot/shooter_controller.c" \
  "$repo_root/modules/algorithm/pid.c" "$repo_root/modules/message_center/message_center.c" \
  "$repo_root/bsp/critical/bsp_critical.c" "$repo_root/config/robots/infantry_standard.c" \
  "$repo_root/tests/host/test_control_recovery.c" -lm -o "$test_build/test_control_recovery"

"$cc" $common_flags -I"$repo_root/application/gimbal" \
  "$repo_root/application/gimbal/yaw_reference.c" "$repo_root/tests/host/test_yaw_reference.c" \
  -lm -o "$test_build/test_yaw_reference"
"$test_build/test_yaw_reference"
"$test_build/test_control_recovery"

"$cc" $common_flags -Wno-unused-parameter \
  -DROBOT_TYPE_infantry_standard -I"$repo_root/config" -I"$repo_root/config/robots" \
  -I"$repo_root/services/motor" -I"$repo_root/core/motor" \
  -I"$repo_root/tests/host/stubs/remote" \
  -I"$repo_root/third_party/nyush_remote/bsp/usart" \
  -I"$repo_root/third_party/nyush_remote/modules/daemon" \
  -I"$repo_root/third_party/nyush_remote/modules/algorithm" \
  -I"$repo_root/bsp/remote/nyush_port" \
  -I"$repo_root/application/cmd" -I"$repo_root/core/contracts" \
  -I"$repo_root/core/common" -I"$repo_root/modules/remote" \
  -I"$repo_root/bsp/can" -I"$repo_root/bsp/remote" -I"$repo_root/bsp/time" -I"$repo_root/modules/message_center" \
  -I"$repo_root/modules/logger" -I"$repo_root/bsp/critical" \
  "$repo_root/bsp/remote/bsp_rc.c" "$repo_root/bsp/remote/nyush_usart.c" \
  "$repo_root/modules/remote/nyush_remote.c" "$repo_root/modules/remote/nyush_daemon.c" \
  "$repo_root/modules/remote/nyush_crc16.c" \
  "$repo_root/application/cmd/cmd_controller.c" "$repo_root/application/cmd/command_router.c" \
  "$repo_root/modules/remote/remote_control.c" "$repo_root/modules/message_center/message_center.c" \
  "$repo_root/bsp/critical/bsp_critical.c" "$repo_root/tests/host/test_remote_chain.c" \
  "$repo_root/config/robots/infantry_standard.c" \
  -lm -o "$test_build/test_remote_chain"
"$test_build/test_remote_chain"

"$cc" $common_flags \
  -I"$repo_root/bsp/critical" \
  -I"$repo_root/modules/message_center" \
  "$repo_root/bsp/critical/bsp_critical.c" \
  "$repo_root/modules/message_center/message_center.c" \
  "$repo_root/tests/host/test_message_center.c" \
  -o "$test_build/test_message_center"

"$cc" $common_flags \
  -DROBOT_TYPE_infantry_standard \
  -I"$repo_root/config" \
  -I"$repo_root/config/robots" \
  -I"$repo_root/core/common" \
  -I"$repo_root/core/chassis" \
  "$repo_root/adapters/chassis/mecanum_chassis_strategy.c" \
  "$repo_root/adapters/chassis/swerve_chassis_strategy.c" \
  "$repo_root/adapters/chassis/omni_chassis_strategy.c" \
  "$repo_root/tests/host/test_chassis_strategies.c" \
  -lm \
  -o "$test_build/test_chassis_strategies"

"$cc" $common_flags -DROBOT_TYPE_infantry_standard \
  -I"$repo_root/tests/host/stubs/can" \
  -I"$repo_root/application/chassis" -I"$repo_root/config" -I"$repo_root/config/robots" \
  -I"$repo_root/core/chassis" -I"$repo_root/core/common" -I"$repo_root/core/contracts" \
  -I"$repo_root/core/motor" -I"$repo_root/services/motor" \
  -I"$repo_root/bsp/can" -I"$repo_root/bsp/time" -I"$repo_root/bsp/critical" \
  -I"$repo_root/modules/can_comm" -I"$repo_root/modules/motor_protocols" \
  -I"$repo_root/modules/message_center" -I"$repo_root/modules/algorithm" \
  -I"$repo_root/modules/logger" \
  "$repo_root/application/chassis/chassis_controller.c" \
  "$repo_root/services/chassis/chassis_strategy_registry.c" \
  "$repo_root/adapters/chassis/omni_chassis_strategy.c" \
  "$repo_root/adapters/chassis/mecanum_chassis_strategy.c" \
  "$repo_root/adapters/chassis/swerve_chassis_strategy.c" \
  "$repo_root/config/robots/infantry_standard.c" \
  "$repo_root/modules/algorithm/pid.c" "$repo_root/modules/can_comm/can_manager.c" \
  "$repo_root/modules/can_comm/motor_registry.c" \
  "$repo_root/modules/message_center/message_center.c" "$repo_root/bsp/critical/bsp_critical.c" \
  "$repo_root/tests/host/test_omni_controller.c" -lm -o "$test_build/test_omni_controller"
"$test_build/test_omni_controller"

"$cc" $common_flags \
  -DROBOT_TYPE_infantry_standard \
  -I"$repo_root/config" \
  -I"$repo_root/config/robots" \
  -I"$repo_root/core/common" \
  -I"$repo_root/core/contracts" \
  -I"$repo_root/core/motor" \
  -I"$repo_root/services/motor" \
  -I"$repo_root/bsp/critical" \
  -I"$repo_root/modules/message_center" \
  "$repo_root/bsp/critical/bsp_critical.c" \
  "$repo_root/modules/message_center/message_center.c" \
  "$repo_root/services/motor/motor_service.c" \
  "$repo_root/tests/host/test_motor_service.c" \
  -o "$test_build/test_motor_service"

"$cc" $common_flags \
  -I"$repo_root/config" \
  -I"$repo_root/core/contracts" \
  -I"$repo_root/modules/message_center" \
  "$repo_root/tests/host/test_contract_sizes.c" \
  -o "$test_build/test_contract_sizes"

"$cc" $common_flags \
  -I"$repo_root/modules/motor_protocols" \
  "$repo_root/modules/motor_protocols/dm_motor_protocol.c" \
  "$repo_root/modules/motor_protocols/lk_motor_protocol.c" \
  "$repo_root/modules/motor_protocols/benmo_motor_protocol.c" \
  "$repo_root/tests/host/test_motor_protocols.c" \
  -lm \
  -o "$test_build/test_motor_protocols"

"$cc" $common_flags \
  -I"$repo_root/application/cmd" \
  -I"$repo_root/config" -I"$repo_root/config/robots" -I"$repo_root/core/chassis" \
  -I"$repo_root/core/common" \
  -I"$repo_root/core/contracts" \
  "$repo_root/application/cmd/command_router.c" \
  "$repo_root/tests/host/test_command_router.c" \
  "$repo_root/adapters/chassis/omni_chassis_strategy.c" \
  "$repo_root/config/robots/infantry_standard.c" \
  -lm \
  -o "$test_build/test_command_router"

"$cc" $common_flags \
  -I"$repo_root/adapters/motor" \
  -I"$repo_root/bsp/can" \
  -I"$repo_root/bsp/time" \
  -I"$repo_root/config" \
  -I"$repo_root/core/common" \
  -I"$repo_root/core/contracts" \
  -I"$repo_root/core/motor" \
  -I"$repo_root/modules/algorithm" \
  -I"$repo_root/modules/motor_protocols" \
  "$repo_root/modules/algorithm/pid.c" \
  "$repo_root/modules/motor_protocols/dm_motor_protocol.c" \
  "$repo_root/modules/motor_protocols/lk_motor_protocol.c" \
  "$repo_root/modules/motor_protocols/benmo_motor_protocol.c" \
  "$repo_root/adapters/motor/vendor_control.c" \
  "$repo_root/adapters/motor/dm_motor_adapter.c" \
  "$repo_root/adapters/motor/lk_motor_adapter.c" \
  "$repo_root/adapters/motor/benmo_motor_adapter.c" \
  "$repo_root/tests/host/test_motor_adapters.c" \
  -lm \
  -o "$test_build/test_motor_adapters"

"$cc" $common_flags -DROBOT_TYPE_infantry_standard \
  -I"$repo_root/tests/host/stubs/can" \
  -I"$repo_root/config" -I"$repo_root/config/robots" \
  -I"$repo_root/core/common" -I"$repo_root/core/contracts" -I"$repo_root/core/motor" \
  -I"$repo_root/bsp/can" -I"$repo_root/bsp/time" -I"$repo_root/bsp/critical" \
  -I"$repo_root/modules/can_comm" -I"$repo_root/modules/motor" \
  -I"$repo_root/modules/motor_protocols" -I"$repo_root/modules/message_center" \
  -I"$repo_root/modules/algorithm" \
  "$repo_root/config/robot_config.c" "$repo_root/config/robots/infantry_standard.c" \
  "$repo_root/config/robots/sentry_swerve.c" \
  "$repo_root/modules/can_comm/can_manager.c" "$repo_root/modules/can_comm/motor_registry.c" \
  "$repo_root/adapters/motor/dji_motor_adapter.c" "$repo_root/modules/algorithm/pid.c" \
  "$repo_root/modules/message_center/message_center.c" "$repo_root/bsp/critical/bsp_critical.c" \
  "$repo_root/tests/host/test_dji_commands.c" -lm -o "$test_build/test_dji_commands"
"$test_build/test_dji_commands"

for robot_type in infantry_standard sentry_swerve; do
  "$cc" $common_flags \
    -D"ROBOT_TYPE_${robot_type}" \
    -I"$repo_root/modules/motor_protocols" \
    -I"$repo_root/config" \
    -I"$repo_root/config/robots" \
    "$repo_root/config/robot_config.c" \
    "$repo_root/config/robots/infantry_standard.c" \
    "$repo_root/config/robots/sentry_swerve.c" \
    "$repo_root/tests/host/test_robot_config.c" \
    -o "$test_build/test_robot_config_${robot_type}"
done

"$test_build/test_message_center"
"$test_build/test_chassis_strategies"
"$test_build/test_motor_service"
"$test_build/test_contract_sizes"
"$test_build/test_motor_protocols"
"$test_build/test_command_router"
"$test_build/test_motor_adapters"
"$test_build/test_robot_config_infantry_standard"
"$test_build/test_robot_config_sentry_swerve"
echo "host tests: PASS"
