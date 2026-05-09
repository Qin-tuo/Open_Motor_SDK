#include "config_loader.hpp"
// 引入 toml++ 头文件 (你需要确保已经下载并放置在正确路径)

#include <algorithm>
#include <cctype>

namespace {

struct HaitaiModelDefaults {
    float pos_max_rad;
    float vel_max_rad_s;
    float torque_max_nm;
};

std::string normalize_model_name(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

HaitaiModelDefaults get_haitai_model_defaults(const std::string& type) {
    const std::string key = normalize_model_name(type);
    if (key == "HT3505-J8") return {95.5f, 32.04f, 0.85f};
    if (key == "HT4310-J10") return {95.5f, 31.42f, 5.8f};
    if (key == "HT6010-J6") return {95.5f, 70.16f, 9.0f};
    return {95.5f, 45.0f, 18.0f};
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
        get_val(m.chan, "chan");
        m.device_name = "can" + std::to_string(m.chan);
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

        if (m.api_type == 7) {
            // Haitai standard-frame protocol. Allow compact config like Type5/6.
            const HaitaiModelDefaults defaults = get_haitai_model_defaults(m.type);
            m.p_min = -defaults.pos_max_rad; m.p_max = defaults.pos_max_rad;
            m.v_min = -defaults.vel_max_rad_s; m.v_max = defaults.vel_max_rad_s;
            m.kp_min = 0.0f; m.kp_max = 500.0f;
            m.kd_min = 0.0f; m.kd_max = 5.0f;
            m.t_min = -defaults.torque_max_nm; m.t_max = defaults.torque_max_nm;
            m.kp_in_use = 20.0f; m.kd_in_use = 0.8f;
            m.pos_min = 0.0f; m.pos_max = 0.0f;
        }

        if (m.api_type == 5 || m.api_type == 6 || m.api_type == 7) {
            // HighTorque/PFL28/Haitai: allow compact config, most fields optional.
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
            get_val(m.kp_in_use, "kp_in_use", false);
            get_val(m.kd_in_use, "kd_in_use", false);
            get_val(m.pos_min, "pos_min", false);
            get_val(m.pos_max, "pos_max", false);
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
