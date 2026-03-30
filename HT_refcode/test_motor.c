#include "test_motor.h"
#include "motor_many.h"




void test_motor_control(const uint8_t id)
{
    const uint8_t mode = 9;
    const data_type_t type = TFLOAT;
    const port_t portx = PORT1;

    switch (mode)
    {
    case 0:
        motor_set_dq_vlot(portx, type, id, 1.5);
        break;
    case 1:
        motor_set_dq_current(portx, type, id, 0.2);
        break;
    case 2:
        motor_set_pos(portx, type, id, 0.1);
        break;
    case 3:
        motor_set_vel(portx, type, id, 0.1f);
        break;
    case 4:
        motor_set_tqe(portx, type, id, 0.5f);
        break;
    case 5:
        motor_set_pos_vel(portx, type, id, 0.1, 0.1);
        break;
    case 6:
        motor_set_pos_vel_MAXtqe(portx, type, id, 0.2, 0.1, NAN);
        break;
    case 7:
        motor_set_pos_velmax_acc(portx, type, id, 0.1, 0.5, 0.1);
        break;
    case 8:
        motor_set_pos_vel_tqe_kp_kd_2(portx, type, id, 2, 0.1, 1, 1, 1);
        break;
    case 9:
        motor_set_brake(portx, type, id);
        break;
    case 10:
        motor_set_stop(portx, type, id);
        break;
    case 11:
        motor_get_state_send(portx, type, id);
        break;   
    case 12:
        motor_get_version(portx, id);
        break;
    default:
        break;
    }
}

