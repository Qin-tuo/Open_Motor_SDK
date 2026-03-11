#include "config_loader.hpp"
// 引入 toml++ 头文件 (你需要确保已经下载并放置在正确路径)

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

        // --- 开始赋值 ---
        
        // 1. Basic Info
        get_val(m.num, "num");
        // string 比较特殊，value<string> 返回的是 string view 或 string
        // 这里的操作稍有不同
        if (auto val = tbl->get("name")->value<std::string>()) m.name = *val; 
        else { std::cerr << "Missing name at index " << index << std::endl; std::exit(-1); }

        if (auto val = tbl->get("type")->value<std::string>()) m.type = *val;
        else { std::cerr << "Missing type at index " << index << std::endl; std::exit(-1); }

        get_val(m.api_type, "api_type");
        
        if (auto val = tbl->get("device")->value<std::string>()) m.device_name = *val;
        else { std::cerr << "Missing device at index " << index << std::endl; std::exit(-1); }

        get_val(m.chan, "chan");
        get_val(m.canid, "canid");

        // 2. Physics Params
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

        // 3. New Fields (新增字段)
        get_val(m.pos_min, "pos_min");
        get_val(m.pos_max, "pos_max");

        motor_list.push_back(m);
    }

    std::cout << "[Config] Successfully loaded " << motor_list.size() << " motors info from TOML." << std::endl;
    return motor_list;
}