#pragma once

#include "config_loader.hpp"
#include "device.hpp"
#include "mapper.hpp"
#include "types.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct MotorCmdVec {
    float p;
    float v;
    float t;
};

class BaseRobot {
public:
    explicit BaseRobot(const std::string& config_file, bool disable_on_destroy = true);
    ~BaseRobot();

    BaseRobot(const BaseRobot&) = delete;
    BaseRobot& operator=(const BaseRobot&) = delete;

    std::size_t MotorCount() const;
    bool GetMotorInfo(int index, Motor_CAN_Info_Struct& info) const;
    bool GetMotorSnapshot(int index, Motor_CAN_Struct& motor) const;
    bool GetCommand_N(int index, MotorCmdVec& command) const;

    bool EnableAll();
    bool DisableAll();
    bool Enable_N(int index);
    bool Disable_N(int index);

    int SetKpd_N(float kp, float kd, int index);
    bool SetKpd_all(float kp, float kd);
    bool ClearError_N(int index);
    bool SetZero_N(int index);
    bool ClearError_All();
    bool SetZero_All();

    bool Stage_N(int index, const MotorCmdVec& target);
    bool Flush_N(int index);
    bool Move_N(int index, const MotorCmdVec& target);
    bool Move(const std::vector<MotorCmdVec>& targets);

    bool SetModes(const std::vector<int>& modes);
    bool SetMode_N(int index, int mode);
    bool ConfigureHaitaiMitLimits_N(int index);
    bool SetModeAll_TypeX(int api_type, int mode);

    bool QueryPos_ALL();
    bool QueryPos_N(int index);
    bool QueryVersion_N(int index);
    int FindMotorIndexByName(const std::string& name) const;
    bool IsMotorReady(int index) const;

    void SetCommandTimeout(std::chrono::milliseconds timeout);
    int CheckCommandTimeouts();

    void PrintStatus() const;
    std::vector<float> GetPosAll() const;
    std::vector<float> GetPosN(int count) const;

private:
    DeviceX* deviceForMotor(int index) const;

    bool disable_on_destroy_ = true;
    std::vector<Motor_CAN_Struct> motors_;
    MotorMapper mapper_;
    std::map<std::string, int> device_name_to_index_;

    // ponytail: one shared lock is enough at current scale; split per motor only if contention is measured.
    mutable std::mutex motor_mutex_;
    std::vector<uint64_t> last_command_ns_;
    std::vector<bool> command_armed_;
    uint64_t command_timeout_ns_ = 100000000ULL;
    std::vector<std::unique_ptr<DeviceX>> devices_;
};
