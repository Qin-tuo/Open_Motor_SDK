#include "config_loader.hpp"
#include "mapper.hpp"
#include "types.hpp"
#if defined(__linux__) || defined(KHCAN_TEST_DEVICE_PROTOCOL)
#include "device.hpp"
#endif

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

int main(int argc, char** argv) {
    const std::string test_suffix = argc > 1
        ? std::filesystem::path(argv[1]).stem().string()
        : "default";
    MotorMapper mapper;
    assert(mapper.add(0, false, 1, 10));
    assert(mapper.add(0, true, 1, 11));
    assert(!mapper.add(0, false, 1, 12));
    assert(mapper.get_id(0, false, 1) == 10);
    assert(mapper.get_id(0, true, 1) == 11);
    assert(mapper.get_id(1, false, 1) == -1);

    assert(motor_mode_supported(1, 0));
    assert(motor_mode_supported(1, 2));
    assert(!motor_mode_supported(1, 1));
    assert(motor_mode_supported(2, 1));
    assert(!motor_mode_supported(2, 0));
    assert(motor_mode_supported(7, 4));
    assert(!motor_mode_supported(7, 5));
    assert(motor_mode_supported(9, 1));
    assert(!motor_mode_supported(9, 0));

    Motor_CAN_Info_Struct info {};
    info.p_min = -2.0f;
    info.p_max = 2.0f;
    info.v_min = -3.0f;
    info.v_max = 3.0f;
    info.kp_min = 0.0f;
    info.kp_max = 100.0f;
    info.kd_min = 0.0f;
    info.kd_max = 10.0f;
    info.t_min = -4.0f;
    info.t_max = 4.0f;
    info.pos_min = -1.0f;
    info.pos_max = 1.0f;

    Motor_CAN_Send_Struct command {};
    command.position = 9.0f;
    command.speed = -9.0f;
    command.torque = 9.0f;
    command.kp = 200.0f;
    command.kd = -1.0f;
    assert(sanitize_motor_command(info, command));
    assert(command.position == 1.0f);
    assert(command.speed == -3.0f);
    assert(command.torque == 4.0f);
    assert(command.kp == 100.0f);
    assert(command.kd == 0.0f);

    command.position = std::numeric_limits<float>::quiet_NaN();
    assert(!sanitize_motor_command(info, command));

    info.api_type = 1;
    info.current_min = -10.0f;
    info.current_max = 10.0f;
    command.mode = 2;
    command.position = 0.0f;
    command.speed = 0.0f;
    command.torque = 0.0f;
    command.kp = 0.0f;
    command.kd = 0.0f;
    assert(!sanitize_motor_command(info, command));

    command.torque = 20.0f;
    assert(sanitize_motor_command(info, command));
    assert(command.torque == 10.0f);

#if defined(__linux__) || defined(KHCAN_TEST_DEVICE_PROTOCOL)
    HaitaiFeedback feedback {};
    const std::array<uint8_t, 8> zero_limits {0xF0, 0, 0, 0, 0, 0, 0, 0};
    assert(!parse_haitai_feedback(1, zero_limits, 7, feedback));

    const std::array<uint8_t, 8> valid_limits {0xF0, 10, 0, 20, 0, 30, 0, 0};
    assert(parse_haitai_feedback(1, valid_limits, 7, feedback));
    assert(feedback.has_mit_limits);
    assert(std::fabs(feedback.mit_pos_max_rad - 1.0f) < 1e-6f);
    assert(std::fabs(feedback.mit_vel_max_rad_s - 0.2f) < 1e-6f);
    assert(std::fabs(feedback.mit_torque_max_nm - 0.3f) < 1e-6f);
#endif

    if (argc > 1) {
        const auto motors = MotorConfigLoader::loadConfig(argv[1]);
        assert(!motors.empty());
        for (const auto& motor : motors) {
            if (motor.type == "EC-A6416-P2-25") {
                assert(motor.p_min == -12.5f && motor.p_max == 12.5f);
                assert(motor.v_min == -18.0f && motor.v_max == 18.0f);
                assert(motor.kp_min == 0.0f && motor.kp_max == 500.0f);
                assert(motor.kd_min == 0.0f && motor.kd_max == 5.0f);
                assert(motor.t_min == -120.0f && motor.t_max == 120.0f);
                assert(motor.current_min == -60.0f && motor.current_max == 60.0f);
                assert(motor.torque_constant == 2.74f);
            }
        }
    }

    const auto model_config =
        std::filesystem::temp_directory_path() /
        ("khcan_encos_models_" + test_suffix + ".toml");
    struct EncosExpected {
        const char* type;
        float kd_max;
        float torque_max;
        float current_max;
        float torque_constant;
    };
    const EncosExpected expected[] = {
        {"EC-A8112-P1-18", 5.0f, 90.0f, 60.0f, 2.1f},
        {"EC-A4310-P2-36", 5.0f, 30.0f, 30.0f, 1.4f},
        {"EC-A4315-P2-36", 5.0f, 70.0f, 30.0f, 2.8f},
        {"EC-A6408-P2-25", 5.0f, 60.0f, 60.0f, 2.35f},
        {"EC-A6416-P2-25", 5.0f, 120.0f, 60.0f, 2.74f},
        {"EC-A10020-P1-12", 50.0f, 150.0f, 70.0f, 2.5f},
        {"EC-A10020-P2-24", 50.0f, 300.0f, 140.0f, 2.6f},
        {"EC-A13715-P1-12.67", 50.0f, 320.0f, 220.0f, 2.5f},
        {"EC-A13720-P1-11.4", 50.0f, 400.0f, 220.0f, 2.5f},
    };
    {
        std::ofstream output(model_config);
        output << "motors = [\n";
        for (std::size_t i = 0; i < std::size(expected); ++i) {
            output << "  { num=" << i + 1 << ", name=\"m" << i + 1
                   << "\", type=\"" << expected[i].type
                   << "\", api_type=8, chan=0, canid=" << i + 1
                   << ", kp_in_use=20, kd_in_use=0.8 }"
                   << (i + 1 == std::size(expected) ? "\n" : ",\n");
        }
        output << "]\n";
    }
    const auto encos_models = MotorConfigLoader::loadConfig(model_config.string());
    assert(encos_models.size() == std::size(expected));
    for (std::size_t i = 0; i < encos_models.size(); ++i) {
        assert(encos_models[i].kd_max == expected[i].kd_max);
        assert(encos_models[i].t_max == expected[i].torque_max);
        assert(encos_models[i].current_max == expected[i].current_max);
        assert(encos_models[i].torque_constant == expected[i].torque_constant);
    }
    std::error_code model_remove_error;
    std::filesystem::remove(model_config, model_remove_error);

    const auto invalid_config =
        std::filesystem::temp_directory_path() /
        ("khcan_duplicate_route_" + test_suffix + ".toml");
    {
        std::ofstream output(invalid_config);
        output << "motors = [\n"
               << "  { num=1, name=\"a\", type=\"X\", api_type=5, chan=0, canid=1 },\n"
               << "  { num=2, name=\"b\", type=\"X\", api_type=5, chan=0, canid=1 }\n"
               << "]\n";
    }
    bool rejected_duplicate = false;
    try {
        (void)MotorConfigLoader::loadConfig(invalid_config.string());
    } catch (const std::runtime_error&) {
        rejected_duplicate = true;
    }
    std::error_code remove_error;
    std::filesystem::remove(invalid_config, remove_error);
    assert(rejected_duplicate);

    const auto invalid_encos_id_config =
        std::filesystem::temp_directory_path() /
        ("khcan_invalid_encos_id_" + test_suffix + ".toml");
    {
        std::ofstream output(invalid_encos_id_config);
        output << "motors = [\n"
               << "  { num=1, name=\"encos\", type=\"EC-A4310-P2-36\", "
               << "api_type=8, chan=0, canid=256, kp_in_use=20, kd_in_use=0.8 }\n"
               << "]\n";
    }
    bool rejected_encos_id = false;
    try {
        (void)MotorConfigLoader::loadConfig(invalid_encos_id_config.string());
    } catch (const std::runtime_error&) {
        rejected_encos_id = true;
    }
    std::error_code encos_id_remove_error;
    std::filesystem::remove(invalid_encos_id_config, encos_id_remove_error);
    assert(rejected_encos_id);
    return 0;
}
