#include "remote_control.h"

#include "bsp_usart.h"
#include "bsp_log.h"
#include "crc16.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"

#define REMOTE_CONTROL_DBUS_FRAME_SIZE 18u
#define REMOTE_CONTROL_VTM_FRAME_SIZE 21u
#define REMOTE_FRAME_SOF1 0xA9u
#define REMOTE_FRAME_SOF2 0x53u
#define REMOTE_FRAME_CRC_OFFSET 19u
#define REMOTE_FRAME_CRC_INIT 0xFFFFu
#define REMOTE_FRAME_CRC_POLY 0x1021u
#define REMOTE_VTM_MODE_INVALID 0xFFu

#define REMOTE_BIT_CH0 16u
#define REMOTE_BIT_CH1 27u
#define REMOTE_BIT_CH2 38u
#define REMOTE_BIT_CH3 49u
#define REMOTE_BIT_MODE_SW 60u
#define REMOTE_BIT_PAUSE 62u
#define REMOTE_BIT_CUSTOM_LEFT 63u
#define REMOTE_BIT_CUSTOM_RIGHT 64u
#define REMOTE_BIT_WHEEL 65u
#define REMOTE_BIT_TRIGGER 76u
#define REMOTE_BIT_MOUSE_X 80u
#define REMOTE_BIT_MOUSE_Y 96u
#define REMOTE_BIT_MOUSE_Z 112u
#define REMOTE_BIT_MOUSE_LEFT 128u
#define REMOTE_BIT_MOUSE_RIGHT 130u
#define REMOTE_BIT_MOUSE_MIDDLE 132u
#define REMOTE_BIT_KEYBOARD 136u

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;
static Remote_Control_Protocol_e rc_protocol = REMOTE_CONTROL_PROTOCOL_UNKNOWN;

static Remote_Control_Protocol_e ResolveProtocol(UART_HandleTypeDef *rc_usart_handle)
{
    if (rc_usart_handle == NULL)
    {
        return REMOTE_CONTROL_PROTOCOL_UNKNOWN;
    }

    if (rc_usart_handle->Instance == USART3)
    {
        return REMOTE_CONTROL_PROTOCOL_DBUS;
    }

    if (rc_usart_handle->Instance == USART1)
    {
        return REMOTE_CONTROL_PROTOCOL_VIDEO_TRANSMITTER;
    }

    return REMOTE_CONTROL_PROTOCOL_UNKNOWN;
}

static uint16_t GetFrameSizeByProtocol(Remote_Control_Protocol_e protocol)
{
    switch (protocol)
    {
    case REMOTE_CONTROL_PROTOCOL_DBUS:
        return REMOTE_CONTROL_DBUS_FRAME_SIZE;
    case REMOTE_CONTROL_PROTOCOL_VIDEO_TRANSMITTER:
        return REMOTE_CONTROL_VTM_FRAME_SIZE;
    default:
        return 0u;
    }
}

static uint32_t ReadBitsU32(const uint8_t *buf, uint16_t bit_offset, uint8_t bit_len)
{
    uint32_t value = 0;
    for (uint8_t i = 0; i < bit_len; ++i)
    {
        uint16_t bit_index = (uint16_t)(bit_offset + i);
        if ((buf[bit_index >> 3] >> (bit_index & 0x07u)) & 0x01u)
        {
            value |= (1u << i);
        }
    }
    return value;
}

static uint8_t ModeSwitchToRCValue(uint8_t mode_sw)
{
    switch (mode_sw)
    {
    case 0u:
        return RC_SW_DOWN;
    case 1u:
        return RC_SW_MID;
    case 2u:
        return RC_SW_UP;
    default:
        return RC_SW_MID;
    }
}

static uint16_t CalcCRC16CCITTFalse(const uint8_t *data, uint16_t len)
{
    uint16_t crc = REMOTE_FRAME_CRC_INIT;

    if (data == NULL)
    {
        return crc;
    }

    for (uint16_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)(data[i] << 8);
        for (uint8_t bit = 0; bit < 8u; ++bit)
        {
            if (crc & 0x8000u)
            {
                crc = (uint16_t)((crc << 1) ^ REMOTE_FRAME_CRC_POLY);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t VerifyRemoteFrame(const uint8_t *frame)
{
    uint16_t legacy_crc;
    uint16_t expected_crc;
    uint16_t recv_crc;

    if (frame == NULL)
    {
        return 0u;
    }

    if (frame[0] != REMOTE_FRAME_SOF1 || frame[1] != REMOTE_FRAME_SOF2)
    {
        return 0u;
    }

    expected_crc = CalcCRC16CCITTFalse(frame, REMOTE_FRAME_CRC_OFFSET);
    legacy_crc = crc_16(frame, REMOTE_FRAME_CRC_OFFSET);
    recv_crc = (uint16_t)(frame[REMOTE_FRAME_CRC_OFFSET] |
                          (frame[REMOTE_FRAME_CRC_OFFSET + 1u] << 8));
    return (uint8_t)((expected_crc == recv_crc) || (legacy_crc == recv_crc));
}

static void UpdateKeyState(uint16_t keyboard_bits)
{
    rc_ctrl[TEMP].key[KEY_PRESS].keys = keyboard_bits;

    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl)
    {
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    }
    else
    {
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    }

    if (rc_ctrl[TEMP].key[KEY_PRESS].shift)
    {
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    }
    else
    {
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    }

    {
        uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
            key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
            key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
            key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      // 当前shift组合键是否按下
            key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
            key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

        for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
        {
            if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            {
                continue;
            }

            if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            {
                rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
            }

            if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            {
                rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
            }

            if ((key_with_shift & j) && !(key_last_with_shift & j))
            {
                rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
            }
        }
    }
}

static void UpdateVTMButtonState()
{
    rc_ctrl[TEMP].vtm.pause_count = rc_ctrl[LAST].vtm.pause_count;
    if (rc_ctrl[TEMP].vtm.pause && !rc_ctrl[LAST].vtm.pause)
    {
        rc_ctrl[TEMP].vtm.pause_count++;
    }
}

static void FinalizeRemoteFrame(void)
{
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
}

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void DBUSFrameToRC(const uint8_t *sbus_buf)
{
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                               //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                        //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                        //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                  // 左侧拨轮
    RectifyRCjoystick();
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2;

    rc_ctrl[TEMP].mouse.x = (int16_t)(sbus_buf[6] | (sbus_buf[7] << 8));
    rc_ctrl[TEMP].mouse.y = (int16_t)(sbus_buf[8] | (sbus_buf[9] << 8));
    rc_ctrl[TEMP].mouse.z = 0;
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];
    rc_ctrl[TEMP].mouse.press_m = 0u;

    rc_ctrl[TEMP].vtm.mode_switch = REMOTE_VTM_MODE_INVALID;
    rc_ctrl[TEMP].vtm.pause = 0u;
    rc_ctrl[TEMP].vtm.custom_left = 0u;
    rc_ctrl[TEMP].vtm.custom_right = 0u;
    rc_ctrl[TEMP].vtm.trigger = 0u;
    rc_ctrl[TEMP].vtm.pause_count = rc_ctrl[LAST].vtm.pause_count;

    UpdateKeyState((uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8)));
    FinalizeRemoteFrame();
}

static void VTMFrameToRC(const uint8_t *frame_buf)
{
    uint8_t mode_sw = (uint8_t)ReadBitsU32(frame_buf, REMOTE_BIT_MODE_SW, 2);

    rc_ctrl[TEMP].rc.rocker_r_ = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_CH0, 11) - RC_CH_VALUE_OFFSET; // Channel 0: 右摇杆水平
    rc_ctrl[TEMP].rc.rocker_r1 = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_CH1, 11) - RC_CH_VALUE_OFFSET; // Channel 1: 右摇杆竖直
    rc_ctrl[TEMP].rc.rocker_l1 = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_CH2, 11) - RC_CH_VALUE_OFFSET; // Channel 2: 左摇杆竖直
    rc_ctrl[TEMP].rc.rocker_l_ = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_CH3, 11) - RC_CH_VALUE_OFFSET; // Channel 3: 左摇杆水平
    rc_ctrl[TEMP].rc.dial = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_WHEEL, 11) - RC_CH_VALUE_OFFSET;
    RectifyRCjoystick();

    rc_ctrl[TEMP].rc.switch_right = ModeSwitchToRCValue(mode_sw);
    rc_ctrl[TEMP].rc.switch_left = ModeSwitchToRCValue(mode_sw);
    rc_ctrl[TEMP].vtm.mode_switch = mode_sw;
    rc_ctrl[TEMP].vtm.pause = (uint8_t)ReadBitsU32(frame_buf, REMOTE_BIT_PAUSE, 1);
    rc_ctrl[TEMP].vtm.custom_left = (uint8_t)ReadBitsU32(frame_buf, REMOTE_BIT_CUSTOM_LEFT, 1);
    rc_ctrl[TEMP].vtm.custom_right = (uint8_t)ReadBitsU32(frame_buf, REMOTE_BIT_CUSTOM_RIGHT, 1);
    rc_ctrl[TEMP].vtm.trigger = (uint8_t)ReadBitsU32(frame_buf, REMOTE_BIT_TRIGGER, 1);
    UpdateVTMButtonState();

    rc_ctrl[TEMP].mouse.x = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_MOUSE_X, 16);
    rc_ctrl[TEMP].mouse.y = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_MOUSE_Y, 16);
    rc_ctrl[TEMP].mouse.z = (int16_t)ReadBitsU32(frame_buf, REMOTE_BIT_MOUSE_Z, 16);
    rc_ctrl[TEMP].mouse.press_l = (uint8_t)(ReadBitsU32(frame_buf, REMOTE_BIT_MOUSE_LEFT, 2) != 0u);
    rc_ctrl[TEMP].mouse.press_r = (uint8_t)(ReadBitsU32(frame_buf, REMOTE_BIT_MOUSE_RIGHT, 2) != 0u);
    rc_ctrl[TEMP].mouse.press_m = (uint8_t)(ReadBitsU32(frame_buf, REMOTE_BIT_MOUSE_MIDDLE, 2) != 0u);

    UpdateKeyState((uint16_t)ReadBitsU32(frame_buf, REMOTE_BIT_KEYBOARD, 16));
    FinalizeRemoteFrame();
}

static uint8_t RemoteControlDecodeFrame(const uint8_t *recv_buf)
{
    if (recv_buf == NULL)
    {
        return 0u;
    }

    switch (rc_protocol)
    {
    case REMOTE_CONTROL_PROTOCOL_DBUS:
        DBUSFrameToRC(recv_buf);
        return 1u;
    case REMOTE_CONTROL_PROTOCOL_VIDEO_TRANSMITTER:
        if (!VerifyRemoteFrame(recv_buf))
        {
            return 0u;
        }
        VTMFrameToRC(recv_buf);
        return 1u;
    default:
        return 0u;
    }
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    if (RemoteControlDecodeFrame(rc_usart_instance->recv_buff))
    {
        DaemonReload(rc_daemon_instance);
    }
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    rc_ctrl[TEMP].vtm.mode_switch = REMOTE_VTM_MODE_INVALID;
    rc_ctrl[LAST].vtm.mode_switch = REMOTE_VTM_MODE_INVALID;
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;

    rc_protocol = ResolveProtocol(rc_usart_handle);
    if (rc_protocol == REMOTE_CONTROL_PROTOCOL_UNKNOWN)
    {
        LOGERROR("[rc] unsupported uart instance for remote control");
        return NULL;
    }

    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = GetFrameSizeByProtocol(rc_protocol);
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    rc_ctrl[TEMP].vtm.mode_switch = REMOTE_VTM_MODE_INVALID;
    rc_ctrl[LAST].vtm.mode_switch = REMOTE_VTM_MODE_INVALID;
    rc_init_flag = 1;
    return rc_ctrl;
}

const RC_ctrl_t *RemoteControlGetData(void)
{
    if (!rc_init_flag)
    {
        return NULL;
    }
    return &rc_ctrl[TEMP];
}

Remote_Control_Protocol_e RemoteControlGetProtocol(void)
{
    if (!rc_init_flag)
    {
        return REMOTE_CONTROL_PROTOCOL_UNKNOWN;
    }
    return rc_protocol;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
