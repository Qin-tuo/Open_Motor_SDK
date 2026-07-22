#include "config_loader.hpp"
#include "toml.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <set>
#include <stdexcept>
#include <tuple>
#include <type_traits>

namespace {

struct MotorModelDefaults {
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
    float current_min;
    float current_max;
    float torque_constant;
};

std::string normalize_model_name(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

bool key_is(const std::string& key, std::initializer_list<const char*> aliases) {
    for (const char* alias : aliases) {
        if (key == alias) return true;
    }
    return false;
}

MotorModelDefaults make_symmetric_defaults(float p_max, float v_max,
                                           float kp_max, float kd_max,
                                           float t_max, float current_max = 0.0f,
                                           float torque_constant = 0.0f) {
    return {-p_max, p_max, -v_max, v_max, 0.0f, kp_max, 0.0f, kd_max,
            -t_max, t_max, -current_max, current_max, torque_constant};
}

MotorModelDefaults make_encos_defaults(float kd_max, float t_max,
                                       float current_max, float torque_constant) {
    return make_symmetric_defaults(12.5f, 18.0f, 500.0f, kd_max, t_max,
                                   current_max, torque_constant);
}

bool get_robstride_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key_is(key, {"RS04", "LZRS04"})) { out = make_symmetric_defaults(12.57f, 15.0f, 5000.0f, 100.0f, 120.0f); return true; }
    if (key_is(key, {"CYBERGEAR", "XIAOMI_CYBERGEAR", "XIAOMI-CYBERGEAR"})) { out = make_symmetric_defaults(12.5f, 30.0f, 500.0f, 5.0f, 12.0f); return true; }
    if (key_is(key, {"RS00", "LZRS00"})) { out = make_symmetric_defaults(12.57f, 33.0f, 500.0f, 5.0f, 14.0f, 16.0f); return true; }
    if (key_is(key, {"RS01", "LZRS01"})) { out = make_symmetric_defaults(12.57f, 44.0f, 500.0f, 5.0f, 17.0f, 23.0f); return true; }
    if (key_is(key, {"RS02", "LZRS02"})) { out = make_symmetric_defaults(12.57f, 44.0f, 500.0f, 5.0f, 17.0f, 23.0f); return true; }
    if (key_is(key, {"RS03", "LZRS03"})) { out = make_symmetric_defaults(12.57f, 20.0f, 5000.0f, 100.0f, 60.0f); return true; }
    if (key_is(key, {"RS05", "LZRS05"})) { out = make_symmetric_defaults(12.57f, 50.0f, 500.0f, 5.0f, 5.5f, 16.0f); return true; }
    if (key_is(key, {"RS06", "LZRS06"})) { out = make_symmetric_defaults(12.57f, 50.0f, 5000.0f, 100.0f, 36.0f); return true; }
    return false;
}

bool get_encos_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key == "EC-A8112-P1-18") { out = make_encos_defaults(5.0f, 90.0f, 60.0f, 2.1f); return true; }
    if (key == "EC-A4310-P2-36") { out = make_encos_defaults(5.0f, 30.0f, 30.0f, 1.4f); return true; }
    if (key == "EC-A4315-P2-36") { out = make_encos_defaults(5.0f, 70.0f, 30.0f, 2.8f); return true; }
    if (key == "EC-A6408-P2-25") { out = make_encos_defaults(5.0f, 60.0f, 60.0f, 2.35f); return true; }
    if (key == "EC-A6416-P2-25") { out = make_encos_defaults(5.0f, 120.0f, 60.0f, 2.74f); return true; }
    if (key == "EC-A10020-P1-12") { out = make_encos_defaults(50.0f, 150.0f, 70.0f, 2.5f); return true; }
    if (key == "EC-A10020-P2-24") { out = make_encos_defaults(50.0f, 300.0f, 140.0f, 2.6f); return true; }
    if (key == "EC-A13715-P1-12.67") { out = make_encos_defaults(50.0f, 320.0f, 220.0f, 2.5f); return true; }
    if (key == "EC-A13720-P1-11.4") { out = make_encos_defaults(50.0f, 400.0f, 220.0f, 2.5f); return true; }
    return false;
}

bool get_dm_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key == "DM4310") { out = make_symmetric_defaults(12.5f, 30.0f, 500.0f, 5.0f, 10.0f); return true; }
    if (key_is(key, {"DM4310_48V", "DM4310-48V", "4310_48V"})) { out = make_symmetric_defaults(12.5f, 50.0f, 500.0f, 5.0f, 10.0f); return true; }
    if (key == "DM4340") { out = make_symmetric_defaults(12.5f, 8.0f, 500.0f, 5.0f, 28.0f); return true; }
    if (key_is(key, {"DM4340_48V", "DM4340-48V"})) { out = make_symmetric_defaults(12.5f, 10.0f, 500.0f, 5.0f, 28.0f); return true; }
    if (key == "DM6006") { out = make_symmetric_defaults(12.5f, 45.0f, 500.0f, 5.0f, 20.0f); return true; }
    if (key == "DM8006") { out = make_symmetric_defaults(12.5f, 45.0f, 500.0f, 5.0f, 40.0f); return true; }
    if (key_is(key, {"DM8009", "DMJ8009P"})) { out = make_symmetric_defaults(12.5f, 45.0f, 500.0f, 5.0f, 54.0f); return true; }
    if (key == "DM10010L") { out = make_symmetric_defaults(12.5f, 25.0f, 500.0f, 5.0f, 200.0f); return true; }
    if (key == "DM10010") { out = make_symmetric_defaults(12.5f, 20.0f, 500.0f, 5.0f, 200.0f); return true; }
    if (key == "DMH3510") { out = make_symmetric_defaults(12.5f, 280.0f, 500.0f, 5.0f, 1.0f); return true; }
    if (key == "DMH6215") { out = make_symmetric_defaults(12.5f, 45.0f, 500.0f, 5.0f, 10.0f); return true; }
    if (key == "DMG6220") { out = make_symmetric_defaults(12.5f, 45.0f, 500.0f, 5.0f, 10.0f); return true; }
    return false;
}

bool get_haitai_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key == "HT2205") { out = make_symmetric_defaults(95.5f, 125.66f, 500.0f, 5.0f, 0.06f); return true; }
    if (key == "HT3505-J8") { out = make_symmetric_defaults(95.5f, 32.04f, 500.0f, 5.0f, 0.85f); return true; }
    if (key_is(key, {"HT4305", "HT4305-J10"})) { out = make_symmetric_defaults(95.5f, 41.89f, 500.0f, 5.0f, 3.0f); return true; }
    if (key == "HT4310-J10") { out = make_symmetric_defaults(95.5f, 31.42f, 500.0f, 5.0f, 1.0f); return true; }
    if (key == "HT6010-J6") { out = make_symmetric_defaults(95.5f, 70.16f, 500.0f, 5.0f, 9.0f); return true; }
    return false;
}

bool get_jc_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key_is(key, {"JC", "JC_SERVO", "JC-SERVO", "JC_SERIES", "JC-SERIES"})) {
        out = make_symmetric_defaults(1.5707964f, 12.566371f, 0.0f, 0.0f, 1.0f);
        return true;
    }
    return false;
}

bool get_model_defaults(int api_type, const std::string& type, MotorModelDefaults& out) {
    const std::string key = normalize_model_name(type);
    if (api_type == 1) return get_robstride_model_defaults(key, out);
    if (api_type == 8) return get_encos_model_defaults(key, out);
    if (api_type == 3) return get_dm_model_defaults(key, out);
    if (api_type == 7) return get_haitai_model_defaults(key, out);
    if (api_type == 9) return get_jc_model_defaults(key, out);
    return false;
}

void apply_model_defaults(Motor_CAN_Info_Struct& motor, const MotorModelDefaults& defaults) {
    motor.p_min = defaults.p_min;
    motor.p_max = defaults.p_max;
    motor.v_min = defaults.v_min;
    motor.v_max = defaults.v_max;
    motor.kp_min = defaults.kp_min;
    motor.kp_max = defaults.kp_max;
    motor.kd_min = defaults.kd_min;
    motor.kd_max = defaults.kd_max;
    motor.t_min = defaults.t_min;
    motor.t_max = defaults.t_max;
    motor.current_min = defaults.current_min;
    motor.current_max = defaults.current_max;
    motor.torque_constant = defaults.torque_constant;
}

bool supports_required_model_defaults(int api_type) {
    return api_type == 1 || api_type == 2 || api_type == 3 ||
           api_type == 7 || api_type == 8 || api_type == 9;
}

bool supported_api_type(int api_type) {
    return api_type == 1 || api_type == 2 || api_type == 3 ||
           api_type == 5 || api_type == 7 || api_type == 8 || api_type == 9;
}

int max_can_id(int api_type) {
    if (api_type == 5 || api_type == 9) return 0x7F;
    return 0xFF;
}

void validate_range(float min, float max, const char* name, int index, bool allow_zero = false) {
    if (!std::isfinite(min) || !std::isfinite(max) ||
        (!(min < max) && !(allow_zero && min == 0.0f && max == 0.0f))) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": invalid " + name + " range");
    }
}

void validate_motor(const Motor_CAN_Info_Struct& motor, int index) {
    if (motor.num <= 0 || motor.name.empty() || motor.type.empty()) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": num, name and type must be set");
    }
    if (!supported_api_type(motor.api_type)) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": unsupported api_type " + std::to_string(motor.api_type));
    }
    if (motor.chan < 0 || motor.canid <= 0 || motor.canid > max_can_id(motor.api_type)) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": invalid CAN channel or ID");
    }
    validate_range(motor.p_min, motor.p_max, "position", index);
    validate_range(motor.v_min, motor.v_max, "velocity", index);
    validate_range(motor.kp_min, motor.kp_max, "kp", index, true);
    validate_range(motor.kd_min, motor.kd_max, "kd", index, true);
    validate_range(motor.t_min, motor.t_max, "torque", index);
    validate_range(motor.current_min, motor.current_max, "current", index, true);
    validate_range(motor.pos_min, motor.pos_max, "joint position", index, true);
    if (!std::isfinite(motor.kp_in_use) || !std::isfinite(motor.kd_in_use) ||
        !std::isfinite(motor.torque_constant) || motor.torque_constant < 0.0f) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": gains and torque constant must be valid");
    }
    if (motor.initial_mode >= 0 &&
        !motor_mode_supported(motor.api_type, motor.initial_mode)) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": initial_mode is not supported by api_type " +
                                 std::to_string(motor.api_type));
    }
    if (motor.kp_in_use < motor.kp_min || motor.kp_in_use > motor.kp_max ||
        motor.kd_in_use < motor.kd_min || motor.kd_in_use > motor.kd_max) {
        throw std::runtime_error("Motor entry " + std::to_string(index) +
                                 ": configured gains are outside their ranges");
    }
}

}  // namespace

// toml++ 不需要手动写 trim，也不需要 split，逻辑会大幅简化
std::vector<Motor_CAN_Info_Struct> MotorConfigLoader::loadConfig(const std::string& filename) {
    std::vector<Motor_CAN_Info_Struct> motor_list;

    // 1. 解析 TOML 文件
    toml::table config;
    try {
        config = toml::parse_file(filename);
    } catch (const toml::parse_error& err) {
        throw std::runtime_error("Failed to parse " + filename + ": " +
                                 std::string(err.description()));
    }

    // 2. 获取 'motors' 数组
    // as_array() 返回一个指针，如果不存在或类型不对则为 nullptr
    auto* motors_arr = config["motors"].as_array();
    
    if (!motors_arr) {
        throw std::runtime_error("'motors' array not found in " + filename);
    }

    // 3. 遍历数组
    std::set<int> motor_numbers;
    std::set<std::string> motor_names;
    std::set<std::tuple<int, bool, int>> motor_routes;
    int index = 0;
    for (auto&& elem : *motors_arr) {
        index++;
        // 确保每个元素是一个 Table (即 {})
        auto* tbl = elem.as_table();
        if (!tbl) {
            throw std::runtime_error("Motor entry " + std::to_string(index) +
                                     " is not an inline table");
        }

        Motor_CAN_Info_Struct m {};

        // 辅助 lambda: 也是为了统一报错处理，模仿你之前的风格
        // value_or 可以提供默认值，但为了严格检查，我们用 value<T>() 并配合 optional
        auto get_val = [&](auto& output, const char* key, bool required = true) {
            // toml++ 取值会自动处理类型转换 (比如 int 写成 50.0 也可以读)
            auto node = tbl->get(key);
            if (!node && required) {
                throw std::runtime_error("Motor entry " + std::to_string(index) +
                                         ": missing key '" + key + "'");
            }
            if (node) {
                // value_or 在节点不存在或类型无法转换时返回默认值
                // value<T>() 返回 std::optional
                auto val = node->value<std::decay_t<decltype(output)>>();
                if (!val) {
                    throw std::runtime_error("Motor entry " + std::to_string(index) +
                                             ": invalid type for '" + key + "'");
                }
                if (val) output = *val;
            }
        };
        auto get_str = [&](std::string& output, const char* key, bool required = true) {
            auto node = tbl->get(key);
            if (!node) {
                if (required) {
                    throw std::runtime_error("Motor entry " + std::to_string(index) +
                                             ": missing key '" + key + "'");
                }
                return;
            }

            auto val = node->value<std::string>();
            if (!val) {
                throw std::runtime_error("Motor entry " + std::to_string(index) +
                                         ": invalid type for '" + key + "'");
            }
            output = *val;
        };

        // --- 开始赋值 ---
        
        // 1. Basic Info
        get_val(m.num, "num");
        get_str(m.name, "name");
        get_str(m.type, "type");

        get_val(m.api_type, "api_type");
        get_val(m.chan, "chan");
        m.device_name = "can" + std::to_string(m.chan);
        get_val(m.canid, "canid");

        // Defaults for optional fields (especially for api_type=5 minimal config).
        m.p_min = -12.57f; m.p_max = 12.57f;
        m.v_min = -50.0f;  m.v_max = 50.0f;
        m.kp_min = 0.0f;   m.kp_max = 5000.0f;
        m.kd_min = 0.0f;   m.kd_max = 100.0f;
        m.t_min = -36.0f;  m.t_max = 36.0f;
        m.current_min = 0.0f; m.current_max = 0.0f;
        m.torque_constant = 0.0f;
        m.kp_in_use = 20.0f; m.kd_in_use = 0.8f;
        m.pos_min = 0.0f;  m.pos_max = 0.0f;
        m.initial_mode = -1;

        if (m.api_type == 9) {
            m.kp_in_use = 0.0f;
            m.kd_in_use = 0.0f;
        }

        bool has_model_defaults = false;
        if (supports_required_model_defaults(m.api_type)) {
            MotorModelDefaults defaults {};
            has_model_defaults = get_model_defaults(m.api_type, m.type, defaults);
            if (has_model_defaults) {
                apply_model_defaults(m, defaults);
                m.pos_min = 0.0f;
                m.pos_max = 0.0f;
            }
        }

        if (m.api_type == 5 || supports_required_model_defaults(m.api_type)) {
            // Known model defaults allow compact config; explicit TOML values still override.
            const bool compact_mit_config = (m.api_type == 5 || has_model_defaults);
            get_val(m.p_min, "p_min", false);
            get_val(m.p_max, "p_max", false);
            get_val(m.v_min, "v_min", false);
            get_val(m.v_max, "v_max", false);
            get_val(m.kp_min, "kp_min", false);
            get_val(m.kp_max, "kp_max", false);
            get_val(m.kd_min, "kd_min", false);
            get_val(m.kd_max, "kd_max", false);
            get_val(m.t_min, "t_min", false);
            get_val(m.t_max, "t_max", false);
            get_val(m.current_min, "current_min", false);
            get_val(m.current_max, "current_max", false);
            get_val(m.torque_constant, "torque_constant", false);
            const bool gains_required = supports_required_model_defaults(m.api_type) &&
                m.api_type != 9;
            get_val(m.kp_in_use, "kp_in_use", gains_required);
            get_val(m.kd_in_use, "kd_in_use", gains_required);
            get_val(m.pos_min, "pos_min", false);
            get_val(m.pos_max, "pos_max", false);
            get_val(m.initial_mode, "initial_mode", false);

            if (!compact_mit_config) {
                get_val(m.p_min, "p_min");
                get_val(m.p_max, "p_max");
                get_val(m.v_min, "v_min");
                get_val(m.v_max, "v_max");
                get_val(m.kp_min, "kp_min");
                get_val(m.kp_max, "kp_max");
                get_val(m.kd_min, "kd_min");
                get_val(m.kd_max, "kd_max");
                get_val(m.t_min, "t_min");
                get_val(m.t_max, "t_max");
            }
        } else {
            // Legacy protocols keep strict required fields.
            get_val(m.p_min, "p_min");
            get_val(m.p_max, "p_max");
            get_val(m.v_min, "v_min");
            get_val(m.v_max, "v_max");
            get_val(m.kp_min, "kp_min");
            get_val(m.kp_max, "kp_max");
            get_val(m.kd_min, "kd_min");
            get_val(m.kd_max, "kd_max");
            get_val(m.t_min, "t_min");
            get_val(m.t_max, "t_max");
            get_val(m.kp_in_use, "kp_in_use");
            get_val(m.kd_in_use, "kd_in_use");
            get_val(m.pos_min, "pos_min");
            get_val(m.pos_max, "pos_max");
            get_val(m.current_min, "current_min", false);
            get_val(m.current_max, "current_max", false);
            get_val(m.torque_constant, "torque_constant", false);
            get_val(m.initial_mode, "initial_mode", false);
        }

        validate_motor(m, index);
        if (!motor_numbers.insert(m.num).second) {
            throw std::runtime_error("Duplicate motor num " + std::to_string(m.num));
        }
        if (!motor_names.insert(m.name).second) {
            throw std::runtime_error("Duplicate motor name '" + m.name + "'");
        }
        const auto route = std::make_tuple(
            m.chan, motor_uses_extended_frame(m.api_type), m.canid);
        if (!motor_routes.insert(route).second) {
            throw std::runtime_error("Duplicate CAN route for motor '" + m.name + "'");
        }
        motor_list.push_back(m);
    }

    if (motor_list.empty()) {
        throw std::runtime_error("No motors configured in " + filename);
    }
    std::cout << "[Config] Successfully loaded " << motor_list.size() << " motors info from TOML." << std::endl;
    return motor_list;
}
