/*
 * 根据电机厂商返回对应的适配实现。
 * 未识别的厂商返回空指针，由上层转成明确错误。
 */
#include "motor_adapter.h"

const MotorAdapterOps *MotorAdapter_Get(MotorVendor_e vendor)
{
    switch (vendor) {
        case MOTOR_VENDOR_DJI:
            return DjiMotorAdapter_Get();
        case MOTOR_VENDOR_DM:
            return DmMotorAdapter_Get();
        case MOTOR_VENDOR_BENMO:
            return BenmoMotorAdapter_Get();
        case MOTOR_VENDOR_LK:
            return LkMotorAdapter_Get();
        default:
            return 0;
    }
}
