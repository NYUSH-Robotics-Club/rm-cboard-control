cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME NYUSH_Infantry)

# ------------------------------------------------------------
# Robot configuration type
# (Override from CLI or VS Code task with -DROBOT_TYPE=...)
# ------------------------------------------------------------
set(ROBOT_TYPE "infantry_standard" CACHE STRING "Robot configuration type")

# Optional: show available values in GUIs (ccmake/cmake-gui)
set_property(CACHE ROBOT_TYPE PROPERTY STRINGS
    infantry_standard
    sentry_swerve
)

# Optional: validate to fail fast on typos
set(_valid_robot_types infantry_standard sentry_swerve)
list(FIND _valid_robot_types "${ROBOT_TYPE}" _robot_type_idx)
if(_robot_type_idx EQUAL -1)
    message(FATAL_ERROR "Invalid ROBOT_TYPE='${ROBOT_TYPE}'. Valid: ${_valid_robot_types}")
endif()

message(STATUS "Selected ROBOT_TYPE: ${ROBOT_TYPE}")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ------------------------------------------------------------
# Executable target
# ------------------------------------------------------------
add_executable(${CMAKE_PROJECT_NAME})

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-u,_printf_float
    -mthumb
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/ST/ARM/DSP/Lib
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/buzzer.c
    third_party/SEGGER/RTT/SEGGER_RTT.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Inc
    config
    config/robots
    core/common
    core/contracts
    core/chassis
    core/motor
    core/vision
    services/motor
    services/chassis
    adapters/motor
    adapters/chassis
    adapters/vision
    bsp/can
    bsp/alarm
    bsp/critical
    bsp/imu
    bsp/remote
    bsp/time
    bsp/uart
    bsp/usb
    modules
    modules/message_center
    modules/can_comm
    modules/debug_print
    modules/algorithm
    modules/motor
    modules/remote
    modules/imu
    application
    application/chassis
    application/shoot
    application/gimbal
    application/cmd
    application/diagnostics
    application/dashboard
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
    Middlewares/ST/ARM/DSP/Include
    third_party/SEGGER/RTT
    third_party/SEGGER/Config
)

# Add project symbols (macros) for executable
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ROBOT_TYPE_${ROBOT_TYPE}
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mthumb
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ------------------------------------------------------------
# Layered static libraries
# ------------------------------------------------------------

# Configuration data is isolated from modules so headers expose declarations
# rather than defining one static copy per translation unit.
add_library(robot_config STATIC
    config/robot_config.c
    config/robots/infantry_standard.c
    config/robots/sentry_swerve.c
)

target_include_directories(robot_config PUBLIC
    config
    config/robots
)
target_compile_definitions(robot_config PRIVATE ROBOT_TYPE_${ROBOT_TYPE})
target_compile_options(robot_config PUBLIC -mthumb)

# Infrastructure has no dependency on motor, chassis, or vision domains.
add_library(framework STATIC
    modules/message_center/message_center.c
)

target_include_directories(framework PUBLIC
    core/common
    core/contracts
    core/chassis
    core/motor
    core/vision
    modules/message_center
    config
)
target_compile_options(framework PUBLIC -mthumb)

# Board support owns MCU/HAL-specific I/O used by reusable modules.
add_library(bsp STATIC
    bsp/can/bsp_can.c
    bsp/alarm/bsp_alarm.c
    bsp/critical/bsp_critical.c
    bsp/imu/bsp_bmi088.c
    bsp/remote/bsp_rc.c
    bsp/remote/nyush_usart.c
    bsp/time/bsp_time.c
    bsp/uart/bsp_uart.c
    bsp/usb/bsp_usb.c
)

target_include_directories(bsp PUBLIC
    bsp/can
    bsp/alarm
    bsp/critical
    bsp/imu
    bsp/remote
    bsp/time
    bsp/uart
    bsp/usb
)
target_include_directories(bsp PRIVATE
    third_party/nyush_remote/bsp/usart
    bsp/remote/nyush_port
)
target_link_libraries(bsp PUBLIC stm32cubemx)
target_compile_options(bsp PUBLIC -mthumb)

# The message center gets its short critical section from BSP instead of
# manipulating MCU interrupt registers itself.
target_link_libraries(framework PUBLIC bsp)

# Existing device/protocol modules. These remain behind upper-layer services.
add_library(modules STATIC
    modules/algorithm/pid.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/QuaternionEKF.c
    modules/motor/motor_driver.c
    modules/motor_protocols/dm_motor_protocol.c
    modules/motor_protocols/lk_motor_protocol.c
    modules/motor_protocols/benmo_motor_protocol.c
    modules/remote/remote_control.c
    modules/remote/nyush_remote.c
    modules/remote/nyush_daemon.c
    modules/remote/nyush_crc16.c
    modules/imu/bmi088driver.c
    modules/imu/gyro_data.c
    modules/imu/wt61c.c
    modules/can_comm/can_manager.c
    modules/can_comm/motor_registry.c
    modules/debug_print/printing.c
    modules/logger/logger.c
    modules/vision_comm/crc8.c
    modules/vision_comm/crc16.c
    modules/vision_comm/seasky_protocol.c
    modules/vision_comm/vision_comm.c
    modules/vision_comm/vision_config.c
    modules/vision_comm/vision_cmd.c
)

target_include_directories(modules PUBLIC
    config
    config/robots
    core/common
    core/contracts
    modules
    modules/message_center
    modules/can_comm
    modules/debug_print
    modules/algorithm
    modules/motor
    modules/motor_protocols
    modules/remote
    modules/imu
    modules/logger
    modules/vision_comm
    Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
    Middlewares/ST/ARM/DSP/Include
    third_party/SEGGER/RTT
    third_party/SEGGER/Config
)

set_property(SOURCE
    modules/remote/nyush_remote.c
    modules/remote/nyush_daemon.c
    modules/remote/nyush_crc16.c
    APPEND PROPERTY INCLUDE_DIRECTORIES
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nyush_remote/bsp/usart"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nyush_remote/modules/daemon"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nyush_remote/modules/algorithm"
    "${CMAKE_CURRENT_SOURCE_DIR}/bsp/remote/nyush_port"
)

# Apply ROBOT_TYPE macro to modules compilation units too
target_compile_definitions(modules PRIVATE
    ROBOT_TYPE_${ROBOT_TYPE}
)

# Inherit STM32CubeMX interface includes and compile definitions (e.g., USE_HAL_DRIVER, STM32F407xx)
target_link_libraries(modules PUBLIC stm32cubemx framework robot_config bsp)
target_compile_options(modules PUBLIC -mthumb)

# Vendor, chassis, and vision adapters plus application-facing services.
add_library(services STATIC
    services/motor/motor_service.c
    services/chassis/chassis_strategy_registry.c
    services/vision/vision_service.c
    adapters/motor/motor_adapter_registry.c
    adapters/motor/vendor_control.c
    adapters/motor/dji_motor_adapter.c
    adapters/motor/dm_motor_adapter.c
    adapters/motor/benmo_motor_adapter.c
    adapters/motor/lk_motor_adapter.c
    adapters/chassis/mecanum_chassis_strategy.c
    adapters/chassis/swerve_chassis_strategy.c
    adapters/chassis/omni_chassis_strategy.c
    adapters/vision/legacy_vision_bridge.c
    adapters/vision/jetson_vision_port.c
)

target_include_directories(services PUBLIC
    config
    config/robots
    core/common
    core/contracts
    core/chassis
    core/motor
    core/vision
    services/motor
    services/chassis
    services/vision
    adapters/motor
    adapters/chassis
)

target_include_directories(services PRIVATE
    bsp/can
    bsp/alarm
    modules/algorithm
    modules/motor
    modules/motor_protocols
    modules/message_center
    modules/vision_comm
    adapters/vision
)

target_compile_definitions(services PRIVATE ROBOT_TYPE_${ROBOT_TYPE})
target_link_libraries(services PUBLIC modules framework robot_config)
target_compile_options(services PUBLIC -mthumb)

# ------------------------------------------------------------
# application library (select chassis vs sentry at configure-time)
# ------------------------------------------------------------

# Common application sources (always built)
set(APPLICATION_SOURCES
    application/cmd/cmd_controller.c
    application/diagnostics/motor_offline_alarm.c
    application/cmd/command_router.c
    application/runtime/app_manifest.c
    application/runtime/app_runtime.c
    application/shoot/shooter_controller.c
    application/gimbal/gimbal_controller.c
    application/gimbal/yaw_reference.c
    application/dashboard/dashboard.c
)

# Pick ONE chassis implementation based on ROBOT_TYPE
# - infantry_standard -> chassis_controller
# - sentry_swerve     -> sentry_controller (and do NOT compile chassis_controller)
if(ROBOT_TYPE STREQUAL "sentry_swerve")
    list(APPEND APPLICATION_SOURCES
        application/chassis/sentry_controller.c
    )
    message(STATUS "Chassis implementation: sentry_controller.c")
else()
    list(APPEND APPLICATION_SOURCES
        application/chassis/chassis_controller.c
    )
    message(STATUS "Chassis implementation: chassis_controller.c")
endif()

add_library(application STATIC ${APPLICATION_SOURCES})

target_include_directories(application PUBLIC
    config
    config/robots
    core/common
    core/contracts
    core/chassis
    core/motor
    core/vision
    services/motor
    services/chassis
    services/vision
    application
    application/cmd
    application/diagnostics
    application/dashboard
    application/runtime
    application/chassis
    application/shoot
    application/gimbal
    modules
    modules/message_center
    modules/can_comm
    modules/algorithm
    modules/motor
    modules/remote
    modules/imu
    modules/vision_comm
    Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
)

target_compile_definitions(application PRIVATE
    ROBOT_TYPE_${ROBOT_TYPE}
)

target_link_libraries(application PUBLIC services modules framework robot_config bsp)
target_compile_options(application PUBLIC -mthumb)

# FreeRTOS is vendored at a fixed version so firmware builds are reproducible.
add_library(freertos_kernel STATIC
    Middlewares/Third_Party/FreeRTOS-Kernel/list.c
    Middlewares/Third_Party/FreeRTOS-Kernel/queue.c
    Middlewares/Third_Party/FreeRTOS-Kernel/tasks.c
    Middlewares/Third_Party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c
)
target_include_directories(freertos_kernel PUBLIC
    runtime/rtos
    Middlewares/Third_Party/FreeRTOS-Kernel/include
    Middlewares/Third_Party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F
)
target_link_libraries(freertos_kernel PUBLIC stm32cubemx)
target_compile_options(freertos_kernel PUBLIC -mthumb)

# Runtime composition is the only layer that knows both RTOS and application APIs.
add_library(robot_runtime STATIC
    runtime/rtos/robot_rtos.c
)
target_include_directories(robot_runtime PUBLIC
    runtime/rtos
)
target_link_libraries(robot_runtime PUBLIC
    freertos_kernel
    application
    modules
)
target_compile_options(robot_runtime PUBLIC -mthumb)


# ------------------------------------------------------------
# Link executable to libraries
# ------------------------------------------------------------
target_link_libraries(${CMAKE_PROJECT_NAME}
    robot_runtime
    freertos_kernel
    application
    services
    modules
    framework
    bsp
    robot_config
    stm32cubemx
    m
    arm_cortexM4lf_math
)
