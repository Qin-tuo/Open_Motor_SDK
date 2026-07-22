#pragma once

#include "mapper.hpp"
#include "types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct HaitaiCommandFrame {
    uint32_t can_id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

struct HaitaiFeedback {
    bool has_position = false;
    bool has_speed = false;
    bool has_current = false;
    bool has_torque = false;
    bool has_temperature = false;
    bool has_status = false;
    bool has_version = false;
    bool has_mit_limits = false;
    bool has_mit_state = false;

    float position_rad = 0.0f;
    float speed_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    float bus_voltage_v = 0.0f;
    float bus_current_a = 0.0f;
    float mit_pos_max_rad = 0.0f;
    float mit_vel_max_rad_s = 0.0f;
    float mit_torque_max_nm = 0.0f;

    uint16_t boot_version = 0;
    uint16_t app_version = 0;
    uint16_t hw_version = 0;
    uint8_t can_proto_version = 0;
    uint8_t mode = 0;
    uint8_t fault = 0;
    uint8_t mit_status = 0;
    bool mit_in_mode = false;
    bool mit_fault = false;
    uint8_t command = 0;
};

HaitaiCommandFrame build_haitai_command(uint8_t mode, float position_rad,
                                        float speed_rad_s, float current_a,
                                        uint32_t can_id = 0x01);

HaitaiCommandFrame build_haitai_mit_command(float position_rad, float speed_rad_s,
                                            float kp, float kd, float torque_nm,
                                            float pos_max_rad,
                                            float vel_max_rad_s,
                                            float torque_max_nm,
                                            uint32_t can_id = 0x01);

HaitaiCommandFrame build_haitai_mit_limits_config(float pos_max_rad,
                                                  float vel_max_rad_s,
                                                  float torque_max_nm,
                                                  uint32_t can_id = 0x01);

HaitaiCommandFrame make_haitai_simple_query(uint8_t cmd, uint32_t can_id);

bool parse_haitai_feedback(uint32_t can_id, const std::array<uint8_t, 8>& data,
                           uint8_t dlc, HaitaiFeedback& out,
                           float mit_pos_max_rad = 95.5f,
                           float mit_vel_max_rad_s = 45.0f,
                           float mit_torque_max_nm = 18.0f);

// Unified SocketCAN device class. Protocol behavior is selected by api_type.
class DeviceX {
protected:
    std::atomic<int> socket_fd{-1};
    std::string iface_name;
    int device_global_index = -1;
    std::vector<Motor_CAN_Struct>* p_motors_data = nullptr;
    MotorMapper* p_mapper = nullptr;
    std::mutex* p_motor_mutex = nullptr;
    std::mutex command_mutex;
    std::atomic<bool> is_running{false};
    std::thread rx_thread;

public:
    ~DeviceX();

    bool Init(const std::string& iface, int dev_idx,
              std::vector<Motor_CAN_Struct>* data_ptr, MotorMapper* mapper_ptr,
              std::mutex* motor_mutex);

    bool SendCommand(int& motor_index);
    bool EnableMotor(int& motor_index);
    bool DisableMotor(int& motor_index);
    bool ClearError(int& motor_index);
    bool SetZero(int& motor_index);
    bool SetMode(int& motor_index, int mode);
    bool ConfigureHaitaiMitLimits(int& motor_index);
    bool QueryPos(int& motor_index);
    bool QueryVersion(int& motor_index);
    unsigned long long EnobufsDropCount() const;
    bool SocketReady() const;
    const std::string& InterfaceName() const;

    void ReceiveLoop();

private:
    bool openSocket(const std::string& iface, bool enable_canfd, int dbitrate = 1000000);
    bool sendFrameWithRetry(const void* frame, std::size_t frame_size, const char* tag);

    bool sendExtendedFrame(uint32_t type, uint16_t data_area, uint8_t motor_id, const uint8_t* data);
    bool writeType1Param(uint8_t motor_id, uint16_t index, float value);
    bool sendExtendedIdFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);
    bool sendExtendedIdFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
    bool sendStandardFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);
    std::atomic<unsigned long long> enobufs_drop_count{0};
    std::atomic<unsigned long long> last_enobufs_log_ms{0};

    // Type 1 (LimX)
    bool EnableMotor_Type1(int& motor_index);
    bool DisableMotor_Type1(int& motor_index);
    bool ClearError_Type1(int& motor_index);
    bool SetZero_Type1(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool SetMode_Type1(int& motor_index, int mode);
    bool SendCommand_Type1(int& motor_index);
    bool QueryPos_Type1(int& motor_index);

    // Type 2 (LK)
    bool EnableMotor_Type2(int& motor_index);
    bool DisableMotor_Type2(int& motor_index);
    bool ClearError_Type2(int& motor_index);
    bool SetZero_Type2(int& motor_index);
    bool SetMode_Type2(int& motor_index, int mode);
    bool SendCommand_Type2(int& motor_index);
    bool QueryPos_Type2(int& motor_index);

    // Type 3 (DM)
    bool EnableMotor_Type3(int& motor_index);
    bool DisableMotor_Type3(int& motor_index);
    bool ClearError_Type3(int& motor_index);
    bool SetZero_Type3(int& motor_index);
    bool SetMode_Type3(int& motor_index, int mode);
    bool SendCommand_Type3(int& motor_index);
    bool QueryPos_Type3(int& motor_index);

    // Type 5 (HighTorque/高擎)
    bool EnableMotor_Type5(int& motor_index);
    bool DisableMotor_Type5(int& motor_index);
    bool ClearError_Type5(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool SetZero_Type5(int& motor_index);
    bool SetMode_Type5(int& motor_index, int mode);
    bool SendCommand_Type5(int& motor_index);
    bool QueryPos_Type5(int& motor_index);

    // Type 7 (Haitai/海泰)
    bool EnableMotor_Type7(int& motor_index);
    bool DisableMotor_Type7(int& motor_index);
    bool ClearError_Type7(int& motor_index);
    bool SetZero_Type7(int& motor_index);
    bool SetMode_Type7(int& motor_index, int mode);
    bool SendCommand_Type7(int& motor_index);
    bool QueryPos_Type7(int& motor_index);
    bool QueryVersion_Type7(int& motor_index);

    // Type 8 (ENCOS EC-A series)
    bool EnableMotor_Type8(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool DisableMotor_Type8(int& motor_index);
    bool ClearError_Type8(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool SetZero_Type8(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool SetMode_Type8(int& motor_index, int mode);
    bool SendCommand_Type8(int& motor_index);
    bool QueryPos_Type8(int& motor_index, std::unique_lock<std::mutex>& lock);

    // Type 9 (JC CAN servo)
    bool EnableMotor_Type9(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool DisableMotor_Type9(int& motor_index);
    bool ClearError_Type9(int& motor_index);
    bool SetZero_Type9(int& motor_index);
    bool SetMode_Type9(int& motor_index, int mode);
    bool SendCommand_Type9(int& motor_index, std::unique_lock<std::mutex>& lock);
    bool QueryPos_Type9(int& motor_index);
};
