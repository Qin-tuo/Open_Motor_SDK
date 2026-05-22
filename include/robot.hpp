#pragma once

#include "config_loader.hpp"
#include "device.hpp"
#include "mapper.hpp"
#include "types.hpp"

#include <map>
#include <set>
#include <string>
#include <vector>

struct MotorCmdVec {
    float p;  // position
    float v;  // velocity
    float t;  // torque/current, depending on api_type
};

class BaseRobot {
private:
    bool disable_on_destroy_ = true;

    inline float Clip(float val, float min, float max) {
        if (min >= max) return val;
        return (val < min) ? min : ((val > max) ? max : val);
    }

public:
    explicit BaseRobot(const std::string& config_file, bool disable_on_destroy = true);
    BaseRobot(const std::string& config_file,
              bool disable_on_destroy,
              const std::vector<std::string>& active_motor_names);
    ~BaseRobot();

    void EnableAll();
    void DisableAll();

    bool Enable_N(int N);
    bool Disable_N(int N);
    int SetKpd_N(float kp, float kd, int N);
    void SetKpd_all(float kp, float kd);

    void ClearError_N(int N);
    void SetZero_N(int N);
    void ClearError_All();
    void SetZero_All();

    bool Stage_N(int N, const MotorCmdVec& target);
    bool Flush_N(int N);
    bool Move_N(int N, const MotorCmdVec& target);
    bool Move(const std::vector<MotorCmdVec>& targets);

    void SetModes(std::vector<int>& modes);
    bool SetMode_N(int N, int mode);
    bool ConfigureHaitaiMitLimits_N(int N);
    void SetModeAll_TypeX(int X, int mode);

    void QueryPos_ALL();
    bool QueryPos_N(int N);
    void QueryVersion_N(int N);
    int FindMotorIndexByName(const std::string& name) const;
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
