#include "config_loader.hpp"
// 引入 toml++ 头文件 (你需要确保已经下载并放置在正确路径)

#include <algorithm>
#include <cctype>
#include <initializer_list>

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
                                           float t_max) {
    return {-p_max, p_max, -v_max, v_max, 0.0f, kp_max, 0.0f, kd_max, -t_max, t_max};
}

bool get_robstride_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key_is(key, {"RS04", "LZRS04"})) { out = make_symmetric_defaults(12.57f, 15.0f, 5000.0f, 100.0f, 120.0f); return true; }
    if (key_is(key, {"CYBERGEAR", "XIAOMI_CYBERGEAR", "XIAOMI-CYBERGEAR"})) { out = make_symmetric_defaults(12.5f, 30.0f, 500.0f, 5.0f, 12.0f); return true; }
    if (key_is(key, {"RS00", "LZRS00"})) { out = make_symmetric_defaults(12.57f, 33.0f, 500.0f, 5.0f, 14.0f); return true; }
    if (key_is(key, {"RS01", "LZRS01"})) { out = make_symmetric_defaults(12.57f, 44.0f, 500.0f, 5.0f, 17.0f); return true; }
    if (key_is(key, {"RS02", "LZRS02"})) { out = make_symmetric_defaults(12.57f, 44.0f, 500.0f, 5.0f, 17.0f); return true; }
    if (key_is(key, {"RS03", "LZRS03"})) { out = make_symmetric_defaults(12.57f, 20.0f, 5000.0f, 100.0f, 60.0f); return true; }
    if (key_is(key, {"RS05", "LZRS05"})) { out = make_symmetric_defaults(12.57f, 50.0f, 500.0f, 5.0f, 5.5f); return true; }
    if (key_is(key, {"RS06", "LZRS06"})) { out = make_symmetric_defaults(12.57f, 50.0f, 5000.0f, 100.0f, 36.0f); return true; }
    return false;
}

bool get_encos_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key == "EC-A8112-P1-18") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 90.0f); return true; }
    if (key == "EC-A4310-P2-36") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 30.0f); return true; }
    if (key == "EC-A6408-P2-25") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 60.0f); return true; }
    if (key == "EC-A10020-P1-12") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 150.0f); return true; }
    if (key == "EC-A10020-P2-24") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 300.0f); return true; }
    if (key == "EC-A13715-P1-12.67") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 320.0f); return true; }
    if (key == "EC-A13720-P1-11.4") { out = make_symmetric_defaults(12.5f, 18.0f, 500.0f, 5.0f, 400.0f); return true; }
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
    if (key == "HT4310-J10") { out = make_symmetric_defaults(95.5f, 31.42f, 500.0f, 5.0f, 5.8f); return true; }
    if (key == "HT6010-J6") { out = make_symmetric_defaults(95.5f, 70.16f, 500.0f, 5.0f, 9.0f); return true; }
    return false;
}

bool get_feetech_model_defaults(const std::string& key, MotorModelDefaults& out) {
    if (key_is(key, {"SCS0037", "SCS0037-C001"})) {
        out = {0.0f, 4.712389f, 0.0f, 19.373f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.18f};
        return true;
    }
    return false;
}

bool get_model_defaults(int api_type, const std::string& type, MotorModelDefaults& out) {
    const std::string key = normalize_model_name(type);
    if (api_type == 1) return get_robstride_model_defaults(key, out);
    if (api_type == 2) return get_encos_model_defaults(key, out);
    if (api_type == 3) return get_dm_model_defaults(key, out);
    if (api_type == 4) return get_feetech_model_defaults(key, out);
    if (api_type == 7) return get_haitai_model_defaults(key, out);
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
}

bool supports_required_model_defaults(int api_type) {
    return api_type == 1 || api_type == 2 || api_type == 3 || api_type == 4 || api_type == 7;
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
        std::cerr << "[Fatal Error] Parsing TOML failed:\n" << err << std::endl;
        std::exit(-1);
    }

    // 2. 获取 'motors' 数组
    // as_array() 返回一个指针，如果不存在或类型不对则为 nullptr
    auto* motors_arr = config["motors"].as_array();
    
    if (!motors_arr) {
        std::cerr << "[Fatal Error] 'motors' array not found in " << filename << std::endl;
        std::exit(-1);
    }

    // 3. 遍历数组
    int index = 0;
    for (auto&& elem : *motors_arr) {
        index++;
        // 确保每个元素是一个 Table (即 {})
        auto* tbl = elem.as_table();
        if (!tbl) {
            std::cerr << "[Warning] Element " << index << " is not a valid inline table, skipping." << std::endl;
            continue;
        }

        Motor_CAN_Info_Struct m;

        // 辅助 lambda: 也是为了统一报错处理，模仿你之前的风格
        // value_or 可以提供默认值，但为了严格检查，我们用 value<T>() 并配合 optional
        auto get_val = [&](auto& output, const char* key, bool required = true) {
            // toml++ 取值会自动处理类型转换 (比如 int 写成 50.0 也可以读)
            auto node = tbl->get(key);
            if (!node && required) {
                 std::cerr << "[Fatal Error] Motor entry " << index 
                           << ": Missing key '" << key << "'" << std::endl;
                 std::exit(-1);
            }
            if (node) {
                // value_or 在节点不存在或类型无法转换时返回默认值
                // value<T>() 返回 std::optional
                auto val = node->value<std::decay_t<decltype(output)>>();
                if (!val && required) {
                    std::cerr << "[Fatal Error] Motor entry " << index 
                              << ": Invalid type for '" << key << "'" << std::endl;
                    std::exit(-1);
                }
                if (val) output = *val;
            }
        };
        auto get_str = [&](std::string& output, const char* key, bool required = true) {
            auto node = tbl->get(key);
            if (!node) {
                if (required) {
                    std::cerr << "[Fatal Error] Motor entry " << index
                              << ": Missing key '" << key << "'" << std::endl;
                    std::exit(-1);
                }
                return;
            }

            auto val = node->value<std::string>();
            if (!val) {
                if (required) {
                    std::cerr << "[Fatal Error] Motor entry " << index
                              << ": Invalid type for '" << key << "'" << std::endl;
                    std::exit(-1);
                }
                return;
            }
            output = *val;
        };

        // --- 开始赋值 ---
        
        // 1. Basic Info
        get_val(m.num, "num");
        get_str(m.name, "name");
        get_str(m.type, "type");

        get_val(m.api_type, "api_type");
        m.chan = 0;
        m.port.clear();
        m.baud = 500000;
        if (m.api_type == 4) {
            get_str(m.port, "port");
            get_val(m.baud, "baud", false);
            m.device_name = m.port;
        } else {
            get_val(m.chan, "chan");
            m.device_name = "can" + std::to_string(m.chan);
        }
        get_val(m.canid, "canid");

        // Defaults for optional fields (especially for api_type=5 minimal config).
        m.p_min = -12.57f; m.p_max = 12.57f;
        m.v_min = -50.0f;  m.v_max = 50.0f;
        m.kp_min = 0.0f;   m.kp_max = 5000.0f;
        m.kd_min = 0.0f;   m.kd_max = 100.0f;
        m.t_min = -36.0f;  m.t_max = 36.0f;
        m.kp_in_use = 20.0f; m.kd_in_use = 0.8f;
        m.pos_min = 0.0f;  m.pos_max = 0.0f;

        if (m.api_type == 6) {
            // PFL28/L28 uses [position(float), current(float)].
            m.p_min = 0.0f;  m.p_max = 9.5f;
            m.v_min = 0.0f;  m.v_max = 0.0f;
            m.kp_min = 0.0f; m.kp_max = 0.0f;
            m.kd_min = 0.0f; m.kd_max = 0.0f;
            m.t_min = 0.0f;  m.t_max = 2.5f;
            m.kp_in_use = 0.0f; m.kd_in_use = 0.0f;
            m.pos_min = 0.0f; m.pos_max = 9.5f;
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

        if (m.api_type == 5 || m.api_type == 6 || supports_required_model_defaults(m.api_type)) {
            // Known model defaults allow compact config; explicit TOML values still override.
            const bool compact_mit_config = (m.api_type == 5 || m.api_type == 6 || has_model_defaults);
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
            get_val(m.kp_in_use, "kp_in_use", supports_required_model_defaults(m.api_type));
            get_val(m.kd_in_use, "kd_in_use", supports_required_model_defaults(m.api_type));
            get_val(m.pos_min, "pos_min", false);
            get_val(m.pos_max, "pos_max", false);

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
        }

        motor_list.push_back(m);
    }

    std::cout << "[Config] Successfully loaded " << motor_list.size() << " motors info from TOML." << std::endl;
    return motor_list;
}
