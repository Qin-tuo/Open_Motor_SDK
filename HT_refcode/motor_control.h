#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H


#include "livelybot_fdcan.h"
#include "convert.h"
#include "motor.h"


void motor_set_dq_vlot(port_t portx, const data_type_t type, const uint8_t id, const float volt);
void motor_set_dq_current(port_t portx, const data_type_t type, const uint8_t id, const float cur);
void motor_set_pos(port_t portx, const data_type_t type, const uint8_t id, const float pos);
void motor_set_vel(port_t portx, const data_type_t type, const uint8_t id, const float vel);
void motor_set_tqe(port_t portx, const data_type_t type, const uint8_t id, const float tqe);
void motor_set_pos_vel(port_t portx, const data_type_t type, const uint8_t id, const float pos, const float vel);
void motor_set_pos_vel_MAXtqe(port_t portx, const data_type_t type, const uint8_t id,
                              const float pos, const float vel, const float tqe);
void motor_set_pos_velmax_acc(port_t portx, const data_type_t type, const uint8_t id, const float pos, const float vel, const float acc);
void motor_set_vel_acc(port_t portx, const data_type_t type, const uint8_t id, const float vel, const float acc);
void motor_set_pos_vel_tqe_kp_kd_2(port_t portx, const data_type_t type, const uint8_t id,
                                   const float pos, const float vel, const float tqe, const float kp, const float kd);

void motor_get_state_send(port_t portx, const data_type_t type, const uint8_t id);
void motor_get_version(port_t portx, const uint8_t id);

void motor_set_stop(port_t portx, const data_type_t type, const uint8_t id);
void motor_set_brake(port_t portx, const data_type_t type, const uint8_t id);
void motor_set_reset(port_t portx, const uint8_t id);

#endif
