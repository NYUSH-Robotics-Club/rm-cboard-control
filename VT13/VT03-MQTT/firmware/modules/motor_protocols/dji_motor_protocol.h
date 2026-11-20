/*
 * Validate DJI command units and GM6020 group/slot mappings without hardware I/O.
 * GM6020 v1.4: voltage +/-25000; current +/-16384 corresponds to +/-3 A.
 */
#ifndef DJI_MOTOR_PROTOCOL_H
#define DJI_MOTOR_PROTOCOL_H

#include "config_types.h"
#include <stdbool.h>
#include <stddef.h>

/* Return zero for invalid mode, limit, or GM6020 address/slot combinations. */
static inline int16_t DjiMotor_CommandLimit(const MotorConfig_t *motor)
{
    if (!motor || motor->vendor != MOTOR_VENDOR_DJI) return 0;
    if (motor->type == MOTOR_TYPE_M3508 || motor->type == MOTOR_TYPE_M2006)
        return 16384;
    if (motor->type != MOTOR_TYPE_GM6020) return 0;

    bool current = motor->protocol.dji.gm6020_mode == GM6020_COMMAND_CURRENT;
    if (!current && motor->protocol.dji.gm6020_mode != GM6020_COMMAND_VOLTAGE)
        return 0;
    if (motor->can_rx_id < 0x205U || motor->can_rx_id > 0x20BU) return 0;
    uint16_t hardware_id = motor->can_rx_id - 0x204U;
    uint16_t group = hardware_id <= 4U ? 0x1FFU : 0x2FFU;
    uint8_t slot = (uint8_t)(hardware_id <= 4U ? hardware_id - 1U : hardware_id - 5U);
    if (current) --group;
    if (motor->can_tx_id != group || motor->tx_slot != slot) return 0;

    int16_t maximum = current ? 16384 : 25000;
    int16_t configured = motor->protocol.dji.command_limit;
    if (configured < 0 || configured > maximum) return 0;
    return configured == 0 ? maximum : configured;
}

#endif
