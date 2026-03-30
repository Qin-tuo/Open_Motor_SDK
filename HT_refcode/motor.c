#include "motor.h"



/************************************下面为需要修改的部分*******************************************/

static motor_state_s motor_state_port[MOTOR_PORT_NUM][MOTOR_MAX_NUM] =  // 下标 + 1 = 电机 ID
{
    {
        // CAN 通道 PORT1
        {
            // ID = 1
            .model = M4438_30,
        },

        {
            // ID = 2
            .model = M5047_36,
        }
    },

    {
        // CAN 通道 PORT2
        {
            // ID = 1
            .model = M4438_30,
        },

        {
            // ID = 2
            .model = M5047_36,
        }
    },
};


const port_mapping_s port_maping[MOTOR_PORT_NUM] =  // 通道映射表
{
    {
        .port = PORT1,
        .fdcan = &hfdcan1,
        .state = motor_state_port[0],
    },

    {
        .port = PORT2,
        .fdcan = &hfdcan2,
        .state = motor_state_port[1],
    },
};

/*******************************************END***************************************************/


p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (fdcanHandle->Instance == port_maping[i].fdcan->Instance)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


p_motor_state_s motor_get_state_pointer2(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].fdcan;
        }
    }

    MOTOR_ERR();
    return NULL;
}



void motor_print_state()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            motor_state_s *p_motor_state = motor_get_state(portx, id);

            printf("PORT: %d, ID: %2d, mode: %2d, temp: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", portx, id, p_motor_state->mode, p_motor_state->temp,
                   p_motor_state->fault, p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
        }
        printf("\r\n");
    }
}


void motor_print_version()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            const p_version_s p_version = &(motor_get_state(portx, id)->version);

            printf("PORT: %d, ID: %2d, version = %d.%d.%d\r\n", portx, id, p_version->major, p_version->minor, p_version->patch);
        }
        printf("\r\n");
    }
}


static motor_type_t motor_get_model1(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    if (id <= MNULL || id > MOTOR_MAX_NUM)
    {
        MOTOR_ERR();
        return MNULL;
    }

    const p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    return p_motor_state[id - 1].model;
}


motor_type_t motor_get_model2(port_t portx, uint8_t id)
{
    if (id <= MNULL || id > MOTOR_MAX_NUM)
    {
        MOTOR_ERR();
        return MNULL;
    }

    const p_motor_state_s p_motor_state = motor_get_state_pointer2(portx);
    const motor_type_t model = p_motor_state[id - 1].model;
    return model;
}


/**
 * @brief 获取指定端口和ID的电机状态指针
 * @param portx 指定电机所在的端口，可能的值为 PORT1 或 PORT2
 * @param id 电机 ID
 * @return 返回类型为 `p_motor_state_s` 的指针
 */
p_motor_state_s motor_get_state(port_t portx, uint8_t id)
{
    const uint8_t index = id - 1;

    return &(motor_get_state_pointer2(portx)[index]);
}


/**
 * @brief 解析电机返回信息
 * @param fdcanHandle
 * @param id 电机 ID
 * @param p_data fdcan 帧数据指针
 * @param len fdcan 数据长度
 */
static void motor_process_state(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id, const uint8_t *p_data, const uint8_t len)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;

    if (p_data[0] == 0x24 && p_data[1] == 0x04 && p_data[2] == 0x00  // TINT16 解析
            && p_data[11] == 0x21 && p_data[12] == 0x0F)
    {
        int16_t pos = 0;
        int16_t vel = 0;
        int16_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 5, sizeof(int16_t));
        my_memcpy((uint8_t *)&vel, p_data + 7, sizeof(int16_t));
        my_memcpy((uint8_t *)&tqe, p_data + 9, sizeof(int16_t));

        p_motor_state[id_index].mode = p_data[3];
        p_motor_state[id_index].position = pos_int2float(pos, TINT16);
        p_motor_state[id_index].velocity = vel_int2float(vel, TINT16);
        const float tqe_temp = tqe_int2float(tqe, TINT16);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
        p_motor_state[id_index].fault = (uint8_t)p_data[13];
    }
    else if (p_data[0] == 0x28 && p_data[1] == 0x04 && p_data[2] == 0x00  // TINT32 解析
             && p_data[19] == 0x21 && p_data[20] == 0x0F)
    {
        int32_t pos = 0;
        int32_t vel = 0;
        int32_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 7, sizeof(int32_t));
        my_memcpy((uint8_t *)&vel, p_data + 11, sizeof(int32_t));
        my_memcpy((uint8_t *)&tqe, p_data + 15, sizeof(int32_t));

        p_motor_state[id_index].mode = p_data[3];
        p_motor_state[id_index].position = pos_int2float(pos, TINT32);
        p_motor_state[id_index].velocity = vel_int2float(vel, TINT32);
        const float tqe_temp = tqe_int2float(tqe, TINT32);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
        p_motor_state[id_index].fault = (uint8_t)p_data[21];
    }
    else if (p_data[0] == 0x2C && p_data[1] == 0x04 && p_data[2] == 0x00  // TFLOAT 解析
             && p_data[19] == 0x21 && p_data[20] == 0x0F)
    {
        float pos = 0;
        float vel = 0;
        float tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 7, sizeof(float));
        my_memcpy((uint8_t *)&vel, p_data + 11, sizeof(float));
        my_memcpy((uint8_t *)&tqe, p_data + 15, sizeof(float));

        p_motor_state[id_index].mode = p_data[3];
        p_motor_state[id_index].position = pos_int2float(pos, TFLOAT);
        p_motor_state[id_index].velocity = vel_int2float(vel, TFLOAT);
        const float tqe_temp = tqe_int2float(tqe, TFLOAT);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
        p_motor_state[id_index].fault = (uint8_t)p_data[21];
    }
    else if (id_index < MOTOR_MAX_NUM && len == 8)   // 一拖多模式解析
    {
        int16_t pos = 0;
        int16_t vel = 0;
        int16_t tqe = 0;

        my_memcpy((uint8_t *)&pos, p_data + 2, sizeof(int16_t));
        my_memcpy((uint8_t *)&vel, p_data + 4, sizeof(int16_t));
        my_memcpy((uint8_t *)&tqe, p_data + 6, sizeof(int16_t));

        p_motor_state[id_index].fault = p_data[1] & 0x3F;
        const uint8_t type = p_data[1] >> 6;

        switch (type)
        {
        case MANY_GET_MODE_FLAUT_POS_VEL_TQE:
            p_motor_state[id_index].mode = p_data[0];
            break;

        case MANY_GET_TEMP_FLAUT_POS_VEL_TQE:
            p_motor_state[id_index].temp = p_data[0];
            break;
        }

        p_motor_state[id_index].position = pos_int2float(pos, TINT16);
        p_motor_state[id_index].velocity = vel_int2float(vel, TINT16);
        const float tqe_temp = tqe_int2float(tqe, TINT16);
        p_motor_state[id_index].torque = tqe_restore(tqe_temp, motor_get_model1(fdcanHandle, id));
    }
    else if (len == 7 && p_data[0] == 0x41 && p_data[1] == 0x01 && p_data[2] == 0x04  // 设置信息解析
             && p_data[3] == 0x4F && p_data[4] == 0x4B && p_data[5] == 0x0D && p_data[6] == 0x0A)
    {
        p_motor_state[id_index].ack = 1;
    }
    else if (p_data[1] == 0xB5 && p_data[2] == 0x02)  // 电机固件版本
    {
        if (len == 5)
        {
            p_motor_state[id_index].version.major = p_data[4] >> 4;
            p_motor_state[id_index].version.minor = p_data[4] & 0x0F | p_data[3] >> 4;
            p_motor_state[id_index].version.patch = p_data[3] & 0x0F;
        }
        else
        {
            p_motor_state[id_index].version.major = 3;
            p_motor_state[id_index].version.minor = 9;
            p_motor_state[id_index].version.patch = 1;
        }
    }
}




static FDCAN_RxHeaderTypeDef fdcan_rx_header;
static uint8_t fdcan_rdata[64] = {0};

/**
 * @brief 解析所有 CAN 通道 FIFO 中的电机状态数据
 *
 */
void motor_process_state_all()
{
    for (int i = 0; i < MOTOR_MAX_NUM; i++)
    {
        while (HAL_FDCAN_GetRxMessage(port_maping[i].fdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata) == HAL_OK)
        {
            if (fdcan_rx_header.DataLength != 0)
            {
                const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

                motor_process_state(port_maping[i].fdcan, fdcan_rx_header.Identifier >> 8, fdcan_rdata, len);
            }
        }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)
    {
        // while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header, fdcan_rdata) == HAL_OK)
        // {
        //     if (fdcan_rx_header.DataLength != 0)
        //     {
        //         const uint16_t len = get_fdcan_data_size(fdcan_rx_header.DataLength);

        //         motor_process_state(hfdcan, fdcan_rx_header.Identifier >> 8, fdcan_rdata, len);
        //     }
        // }
    }
}
