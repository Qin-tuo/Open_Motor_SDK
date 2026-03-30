#include "motor_config.h"


static uint8_t motor_config_closed_loop(void (*action)(FDCAN_HandleTypeDef *, uint8_t), FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;
    uint16_t t = 0;
    const uint16_t t_max = 10;

    p_motor_state[id_index].ack = 0;
    for (int i = 0; i < 10; i++)
    {
        set_pos_rezero(fdcanHandle, id);
        HAL_Delay(100);
        motor_process_state_all();
        if (p_motor_state[id_index].ack != 0)
        {
            break;
        }

        if (++t > t_max)
        {
            return 1;
        }
    }

    return 0;
}


/**
 * @brief 重置电机零位
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 * @return 0-成功，1-重置零位失败，2-保存失败
 */
uint8_t motor_pos_reset(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_reset_int8(fdcanHandle, id);
    set_motor_reset_int8(fdcanHandle, id);
    set_motor_reset_int8(fdcanHandle, id);
    HAL_Delay(100);

    if (motor_config_closed_loop(set_pos_rezero, fdcanHandle, id) != 0)
    {
        return 1;
    }

    if (motor_config_closed_loop(set_conf_write, fdcanHandle, id) != 0)
    {
        return 2;
    }

    set_motor_reset_int8(fdcanHandle, id);
    set_motor_reset_int8(fdcanHandle, id);
    set_motor_reset_int8(fdcanHandle, id);
    HAL_Delay(100);

    return 0;
}


/**
 * @brief 保存电机设置
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 * @return 0-成功，1-失败
 */
uint8_t motor_conf_write(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    if (motor_config_closed_loop(set_conf_write, fdcanHandle, id) != 0)
    {
        return 1;
    }

    set_motor_reset_int8(fdcanHandle, id);
    set_motor_reset_int8(fdcanHandle, id);
    set_motor_reset_int8(fdcanHandle, id);
    HAL_Delay(100);

    return 0;
}

