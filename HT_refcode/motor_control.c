#include "motor_control.h"
#include "motor.h"



/**
 * @brief DQ 电压模式（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V
 */
void motor_set_dq_vlot(port_t portx, const data_type_t type, const uint8_t id, const float volt)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp = vol_float2int(volt, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_volt_float(fdcanHandle, id, temp);
        break;
    case TINT32:
        set_dq_volt_int32(fdcanHandle, id, temp);
        break;
    case TINT16:
        set_dq_volt_int16(fdcanHandle, id, temp);
        break;
    default:
        break;
    }
}


/**
 * @brief DQ 电流模式（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A
 */
void motor_set_dq_current(port_t portx, const data_type_t type, const uint8_t id, const float cur)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp = cur_float2int(cur, type);

    switch(type)
    {
    case TFLOAT:
        set_dq_current_float(fdcanHandle, id, temp);
        break;
    case TINT32:
        set_dq_current_int32(fdcanHandle, id, temp);
        break;
    case TINT16:
        set_dq_current_int16(fdcanHandle, id, temp);
        break;
    default:
        break;
    }
}


/**
 * @brief 位置模式，使用最大速度和加速度运动到目标位置（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos(port_t portx, const data_type_t type, const uint8_t id, const float pos)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float temp2 = pos_float2int(temp1, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_float(fdcanHandle, id, temp2);
        break;
    case TINT32:
        set_pos_int32(fdcanHandle, id, temp2);
        break;
    case TINT16:
        set_pos_int16(fdcanHandle, id, temp2);
        break;
    default:
        break;
    }
}


/**
 * @brief 速度模式，以最大加速度加速到指定速度（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_vel(port_t portx, const data_type_t type, const uint8_t id, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float temp2 = vel_float2int(temp1, type);

    switch(type)
    {
    case TFLOAT:
        set_vel_float(fdcanHandle, id, temp2);
        break;
    case TINT32:
        set_vel_int32(fdcanHandle, id, temp2);
        break;
    case TINT16:
        set_vel_int16(fdcanHandle, id, temp2);
        break;
    default:
        break;
    }
}


/**
 * @brief 力矩模式（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_set_tqe(port_t portx, const data_type_t type, const uint8_t id, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp1 = tqe_adjust(tqe, motor_get_model2(portx, id));
    const float temp2 = tqe_float2int(temp1, type);

    switch(type)
    {
    case TFLOAT:
        set_torque_float(fdcanHandle, id, temp2);
        break;
    case TINT32:
        set_torque_int32(fdcanHandle, id, temp2);
        break;
    case TINT16:
        set_torque_int16(fdcanHandle, id, temp2);
        break;
    default:
        break;
    }
}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，不限制加速度和最大输出力矩（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_vel(port_t portx, const data_type_t type, const uint8_t id, const float pos, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_float(fdcanHandle, id, pos2, vel2, NAN_FLOAT);
        break;
    case TINT32:
        set_pos_vel_tqe_int32(fdcanHandle, id, pos2, vel2, NAN_INT32);
        break;
    case TINT16:
        set_pos_vel_tqe_int16(fdcanHandle, id, pos2, vel2, NAN_INT16);
        break;
    default:
        break;
    }
}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_set_pos_vel_MAXtqe(port_t portx, const data_type_t type, const uint8_t id,
                              const float pos, const float vel, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe1 = tqe_adjust(tqe, motor_get_model2(portx, id));
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);
    const float tqe2 = tqe_float2int(tqe1, type);


    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_float(fdcanHandle, id, pos2, vel2, tqe2);
        break;
    case TINT32:
        set_pos_vel_tqe_int32(fdcanHandle, id, pos2, vel2, tqe2);
        break;
    case TINT16:
        set_pos_vel_tqe_int16(fdcanHandle, id, pos2, vel2, tqe2);
        break;
    default:
        break;
    }
}


/**
 * @brief 位置、速度、加速度模式（梯形控制）（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转/秒2（rps2）、弧度/秒2（rad/s2）、或度/秒2（°/s2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_velmax_acc(port_t portx, const data_type_t type, const uint8_t id, const float pos, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc1 = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);
    const float acc2 = acc_float2int(acc1, type);


    switch(type)
    {
    case TFLOAT:
        set_pos_velmax_acc_float(fdcanHandle, id, pos2, vel2, acc2);
        break;
    case TINT32:
        set_pos_velmax_acc_int32(fdcanHandle, id, pos2, vel2, acc2);
        break;
    case TINT16:
        set_pos_velmax_acc_int16(fdcanHandle, id, pos2, vel2, acc2);
        break;
    default:
        break;
    }
}


/**
 * @brief 速度、加速度模式，以目标加速度加速到目标速度（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转/秒2（rps2）、弧度/秒2（rad/s2）、或度/秒2（°/s2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_vel_acc(port_t portx, const data_type_t type, const uint8_t id, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc1 = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float vel2 = vel_float2int(vel1, type);
    const float acc2 = acc_float2int(acc1, type);

    switch(type)
    {
    case TFLOAT:
        set_vel_acc_float(fdcanHandle, id, vel2, acc2);
        break;
    case TINT32:
        set_vel_acc_int32(fdcanHandle, id, vel2, acc2);
        break;
    case TINT16:
        set_vel_acc_int16(fdcanHandle, id, vel2, acc2);
        break;
    default:
        break;
    }
}


// /**
//  * @brief 运控模式 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩)（并让电机返回状态信息） 不建议使用，建议使用 motor_set_pos_vel_tqe_kp_kd_2
//  * @param fdcanHandle &hfdcanx
//  * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
//  * @param id 电机 ID
//  * @param pos 位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
//  * @param vel 速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
//  * @param tqe 力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
//  * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
//  * @param kd Mkd = kd * 1 (Mkd 表示电机内部 kd)
//  */
// void motor_set_pos_vel_tqe_kp_kd(port_t portx, const data_type_t type, const uint8_t id,
//                                  const float pos, const float vel, const float tqe, const float kp, const float kd)
// {
//     FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
//     const motor_type_t model = motor_get_model2(portx, id);

//     const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
//     const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
//     const float tqe1 = tqe_adjust(tqe, model);
//     const float pos2 = pos_float2int(pos1, type);
//     const float vel2 = vel_float2int(vel1, type);
//     const float tqe2 = tqe_float2int(tqe1, type);

//     const float kp2 = pid_adjust(kp, model);
//     const float kd2 = pid_adjust(kd, model);

//     const float kp3 = pid_float2int(kp2, type);
//     const float kd3 = pid_float2int(kd2, type);

//     switch(type)
//     {
//     case TFLOAT:
//         set_pos_vel_tqe_kp_kd_float(fdcanHandle, id, pos2, vel2, tqe2, kp3, kd3);
//         break;
//     case TINT32:
//         set_pos_vel_tqe_kp_kd_int32(fdcanHandle, id, pos2, vel2, tqe2, kp3, kd3);
//         break;
//     case TINT16:
//         set_pos_vel_tqe_kp_kd_int16(fdcanHandle, id, pos2, vel2, tqe2, kp3, kd3);
//         break;
//     default:
//         break;
//     }
// }


/**
 * @brief 运控模式2 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩)（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kd * 1 (Mkd 表示电机内部 kd)
 */
void motor_set_pos_vel_tqe_kp_kd_2(port_t portx, const data_type_t type, const uint8_t id,
                                   const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const motor_type_t model = motor_get_model2(portx, id);

    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe1 = tqe_adjust(tqe, model);
    const float pos2 = pos_float2int(pos1, type);
    const float vel2 = vel_float2int(vel1, type);
    const float tqe2 = tqe_float2int(tqe1, type);

    const float kp2 = pid_adjust(kp, model);
    const float kd2 = pid_adjust(kd, model);

    const float kp3 = pid_float2int(kp2, type);
    const float kd3 = pid_float2int(kd2, type);

    switch(type)
    {
    case TFLOAT:
        set_pos_vel_tqe_kp_kd_float_2(fdcanHandle, id, pos2, vel2, tqe2, kp3, kd3);
        break;
    case TINT32:
        set_pos_vel_tqe_kp_kd_int32_2(fdcanHandle, id, pos2, vel2, tqe2, kp3, kd3);
        break;
    case TINT16:
        set_pos_vel_tqe_kp_kd_int16_2(fdcanHandle, id, pos2, vel2, tqe2, kp3, kd3);
        break;
    default:
        break;
    }
}


/**
 * @brief 发送查询查询电机状态信息的指令（在motor_process_state中解析）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 */
void motor_get_state_send(port_t portx, const data_type_t type, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    switch(type)
    {
    case TFLOAT:
        read_motor_state_float(fdcanHandle, id);
        break;
    case TINT32:
        read_motor_state_int32(fdcanHandle, id);
        break;
    case TINT16:
        read_motor_state_int16(fdcanHandle, id);
        break;
    default:
        break;
    }
}


/**
 * @brief 发送查询电机固件版本号指令（在motor_process_state中解析）
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_get_version(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    // for (uint8_t i = 0; i < 5; i++)
    {
        read_motor_version_int16(fdcanHandle, id);
    }
}


/**
 * @brief 停止模式，电机三相都断开（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_set_stop(port_t portx, const data_type_t type, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    switch(type)
    {
    case TFLOAT:
        set_motor_stop_float(fdcanHandle, id);
        break;
    case TINT32:
        set_motor_stop_int32(fdcanHandle, id);
        break;
    case TINT16:
        set_motor_stop_int16(fdcanHandle, id);
        break;
    default:
        break;
    }
}


/**
 * @brief 刹车模式（阻尼模式），电机三相都接地（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_set_brake(port_t portx, const data_type_t type, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    switch(type)
    {
    case TFLOAT:
        set_motor_brake_float(fdcanHandle, id);
        break;
    case TINT32:
        set_motor_brake_int32(fdcanHandle, id);
        break;
    case TINT16:
        set_motor_brake_int16(fdcanHandle, id);
        break;
    default:
        break;
    }
}


/**
 * @brief 电机软重启，重启后进入停止模式
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_set_reset(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_reset_int8(fdcanHandle, id);
}





