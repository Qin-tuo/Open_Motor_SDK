#ifndef _MOTOR_MANY_H
#define _MOTOR_MANY_H


#include "convert.h"
#include "my_fdcan.h"
#include "motor.h"


#define  MANY_PORT_SIZE   MOTOR_PORT_NUM  // 通道数量
#define  MANY_MOTOR_SIZE  MOTOR_MAX_NUM  // 一拖多模式下，每个CAN通道控制的电机数量，取值范围为(0，30]


#if MANY_MOTOR_SIZE > 0 && MANY_MOTOR_SIZE <= 30
#define  MANY_DATA_BUF_MAX_LEN   (MANY_MOTOR_SIZE * sizeof(many_pos_vel_tqe_kp_ki_kd_s))
#else
#error "MANY_MOTOR_SIZE value out of range error!!!"
#endif


#define  MODE_POSITION              0X80
#define  MODE_VELOCITY              0X81
#define  MODE_TORQUE                0X82
#define  MODE_VOLTAGE               0X83
#define  MODE_CURRENT               0X84
#define  MODE_TIME_OUT              0x85

#define  MODE_POS_VEL_TQE           0X90
#define  MODE_POS_VEL_TQE_KP_KD     0X93
// #define  MODE_POS_VEL_TQE_KP_KI_KD  0X98  // 弃用
#define  MODE_POS_VEL_KP_KD         0X9E
// #define  MODE_POS_VEL_TQE_RKP_RKD   0XA3  // 弃用
// #define  MODE_POS_VEL_RKP_RKD       0XA8  // 弃用
#define  MODE_POS_VEL_ACC           0XAD
#define  MODE_POS_VEL_TQE_KP_KD2    0XB0



#pragma pack(1)
typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
} many_pos_vel_tqe_s;

typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t acc;
} many_pos_vel_acc_s;

typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
    int16_t kp;
    int16_t kd;
} many_pos_vel_tqe_kp_kd_s;

typedef struct
{
    int16_t pos;
    int16_t vel;
    int16_t tqe;
    int16_t kp;
    int16_t ki;
    int16_t kd;
} many_pos_vel_tqe_kp_ki_kd_s;


typedef struct
{
    union
    {
        int16_t position[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t velocity[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t torque[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t voltage[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t current[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        int16_t timeout[MANY_DATA_BUF_MAX_LEN / sizeof(int16_t)];
        many_pos_vel_tqe_s pos_vel_tqe[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_s)];
        many_pos_vel_acc_s pos_vel_acc[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_acc_s)];
        many_pos_vel_tqe_kp_kd_s pos_vel_tqe_kp_kd[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_kd_s)];
        many_pos_vel_tqe_kp_ki_kd_s pos_vel_tqe_kp_ki_kd[MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_ki_kd_s)];
        uint8_t data[MANY_DATA_BUF_MAX_LEN];
    };
    uint8_t mode;
} many_data_s, *p_many_data_s;
#pragma pack()



void motor_many_dq_volt(port_t portx, const uint8_t id, const float vol);
void motor_many_dq_current(port_t portx, const uint8_t id, const float cur);
void motor_many_pos(port_t portx, const uint8_t id, const float pos);
void motor_many_vel(port_t portx, const uint8_t id, const float vel);
void motor_many_tqe(port_t portx, const uint8_t id, const float tqe);
void motor_many_time_out(port_t portx, const uint8_t id, const int16_t t_ms);
void motor_many_pos_vel(port_t portx, const uint8_t id, const float pos, const float vel);
void motor_many_pos_vel_MAXtqe(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe);
void motor_many_pos_vel_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc);
void motor_many_pos_vel_tqe_kp_kd(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd);
void motor_many_pos_vel_tqe_kp_kd_2(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd);

void motor_many_send(port_t portx, many_request_type_t request_type);

#endif
