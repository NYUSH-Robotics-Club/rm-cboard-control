cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
project(CBoard_Diagnosis C ASM)
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# Select only the frozen initialization needed by this firmware. Generated
# main/interrupts/USB sources remain untouched as reference material.
get_target_property(board_sources ${PROJECT_NAME} SOURCES)
list(FILTER board_sources INCLUDE REGEX "/(gpio|can|stm32f4xx_hal_msp|syscalls|sysmem)\\.c$|startup_stm32f407xx\\.s$")
set_property(TARGET ${PROJECT_NAME} PROPERTY SOURCES ${board_sources})
target_sources(${PROJECT_NAME} PRIVATE
    runtime/diagnosis/main.c
    bsp/diagnosis/diagnosis_can.c
    services/diagnosis/diagnosis.c
    services/diagnosis/diagnosis_terminal.c
    bsp/diagnosis/diagnosis_rtt.c
    third_party/SEGGER/RTT/SEGGER_RTT.c
)
target_include_directories(${PROJECT_NAME} PRIVATE core/diagnosis bsp/diagnosis services/diagnosis
    third_party/SEGGER/RTT third_party/SEGGER/Config)
target_compile_options(${PROJECT_NAME} PRIVATE -Wextra -Werror)
target_link_libraries(${PROJECT_NAME} stm32cubemx)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
