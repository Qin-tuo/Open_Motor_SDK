#pragma once
#include "types.hpp"
#include "mapper.hpp"
#include "device.hpp"
#include "config_loader.hpp"
#include <map>
#include <vector>
#include <string>

/**
 * =============================================================
 * Mode Map Definition (根据源代码逻辑提取)
 * =============================================================
 *
 * 【Type 1: LimX / 灵足协议�?
 * 0 : 运控模式 (Motion Control / MIT)
 * 1 : 位置模式 (Position Mode)
 * 2 : 速度模式 (Speed Mode)
 * 3 : 电流/力矩模式 (Current / Torque Mode)
 *
 * 【Type 2: LK / 宇树旧协议�?
 * 1 : 力矩/混合控制模式 (Torque / MIT) -> 发送指�?0xA1
 * 2 : 位置模式 (Position Mode)        -> 发送指�?0xA3
 * 3 : 速度模式 (Speed Mode)           -> 发送指�?0xA2
 * 其他 : 仅读取状�?(Read Only)        -> 发送指�?0x9C
 *
 * *注意*：Type 1 �?MIT �?0，�?Type 2 �?MIT �?1�?
 * =============================================================
 */

struct MotorCmdVec {
    float p; // 对应 Position
    float v; // 对应 Speed
    float t; // 对应 Torque
};
class BaseRobot {

private:
    inline float Clip(float val, float min, float max) {
            // [极速检查] 
            if (min >= max) { return val;}
            return (val < min) ? min : ((val > max) ? max : val);
        }
public:
    std::vector<Motor_CAN_Struct> global_motors;
    std::vector<DeviceX*> devices;
    TopoMapper mapper;
    std::map<std::string, int> device_name_map_idx; 
public:

    BaseRobot(const std::string& config_file);

    ~BaseRobot();

    void EnableAll();
    void DisableAll();
    
    void Enable_N(int N);
    void Disable_N(int N);
    int SetKpd_N(float kp, float kd,int N);
    void SetKpd_all(float kp, float kd);

    void ClearError_N(int N);
    void SetZero_N(int N);

    void ClearError_All();
    void SetZero_All();

    void Move_N(int N,const MotorCmdVec& target);
// 【新增】合并后的运动控制函�?
    void Move(const std::vector<MotorCmdVec>& targets);
  //  *  mode: 0-运控, 1-位置, 2-速度, 3-电流�?两者统一接口，内部实现已经自动适配
    void SetModes(std::vector<int>& modes);
    void SetMode_N(int N,int mode);
    //  *  mode: 0-运控, 1-位置, 2-速度, 3-电流
    void SetModeAll_TypeX(int X, int mode);
    void QueryPos_ALL();
    void QueryPos_N(int N);

    void PrintStatus();
    std::vector<float> GetPosAll();
    std::vector<float> GetPosN(int n);

};

