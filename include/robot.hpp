#pragma once

#include "config_loader.hpp"
#include "device.hpp"
#include "mapper.hpp"
#include "types.hpp"

#include <map>
#include <string>
#include <vector>

struct MotorCmdVec {
    float p;  // position
    float v;  // velocity
    float t;  // torque/current, depending on api_type
};

class BaseRobot {
private:
    inline float Clip(float val, float min, float max) {
        if (min >= max) return val;
        return (val < min) ? min : ((val > max) ? max : val);
    }

public:
    explicit BaseRobot(const std::string& config_file);
    ~BaseRobot();

    void EnableAll();
    void DisableAll();

    void Enable_N(int N);
    void Disable_N(int N);
    int SetKpd_N(float kp, float kd, int N);
    void SetKpd_all(float kp, float kd);

    void ClearError_N(int N);
    void SetZero_N(int N);
    void ClearError_All();
    void SetZero_All();

    void Move_N(int N, const MotorCmdVec& target);
    void Move(const std::vector<MotorCmdVec>& targets);

    void SetModes(std::vector<int>& modes);
    void SetMode_N(int N, int mode);
    void SetModeAll_TypeX(int X, int mode);

    void QueryPos_ALL();
    void QueryPos_N(int N);
    void QueryVersion_N(int N);
    bool IsMotorReady(int N) const;

    void PrintStatus();
    std::vector<float> GetPosAll();
    std::vector<float> GetPosN(int n);

    std::vector<Motor_CAN_Struct> global_motors;
    std::vector<DeviceX*> devices;
    std::vector<FeetechServoDevice*> feetech_devices;
    TopoMapper mapper;
    std::map<std::string, int> device_name_map_idx;
};
