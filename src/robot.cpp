#include "robot.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

BaseRobot::BaseRobot(const std::string& config_file) {
    auto infos = MotorConfigLoader::loadConfig(config_file);
    if (infos.empty()) { std::cerr << "Error: No motors.\n"; return; }

    int max_chan = 0, max_canid = 0;
    int device_counter = 0;

    // 1. 建立设备名到索引的映射
    for (auto& info : infos) {
        if (device_name_map_idx.find(info.device_name) == device_name_map_idx.end()) {
            device_name_map_idx[info.device_name] = device_counter++;
        }
        if (info.chan > max_chan) max_chan = info.chan;
        if (info.canid > max_canid) max_canid = info.canid;
    }

    // 2. 初始化全局电机向量
    global_motors.resize(infos.size()); 
    std::vector<std::vector<uint>> topo_matrix;
    
    for (int i = 0; i < infos.size(); ++i) {
        auto& info = infos[i];
        
        info.device_index = device_name_map_idx[info.device_name];

        global_motors[i].info = info;
        // 把 CSV 里的默认 kp/kd 填入发送结构体作为初始值
        global_motors[i].send.kd = info.kd_in_use;
        global_motors[i].send.kp = info.kp_in_use;

        topo_matrix.push_back({
            (uint)info.device_index, 
            (uint)info.chan, 
            (uint)info.canid, 
            (uint)i
        });
    }

    // 3. 初始化拓扑映射器
    mapper = TopoMapper({(uint)device_counter, (uint)max_chan + 1, (uint)max_canid + 1}, topo_matrix);

    // 4. 打开所有 USB2CAN 设备句柄
    devices.resize(device_counter, nullptr);

    for(auto const& [name, idx] : device_name_map_idx) {
        std::string full_device_path = "/dev/" + name;
        int32_t handle = openUSBCAN(full_device_path.c_str());
        if (handle == -1) {
            std::cerr << "[Error] Failed to open CAN device: " << full_device_path << std::endl;
        } else {
            std::cout << "[Info] Opened CAN device: " << full_device_path << " (Handle: " << handle << ")" << std::endl;
        }
        open_device_handles[name] = handle;
    }

    // 5. 根据 api_type 实例化对应的设备驱动类 (Type1 or Type2)
    for (int i = 0; i < global_motors.size(); ++i) {
        int d_idx = global_motors[i].info.device_index;
        
        // 如果该设备(USB2CAN卡)还未被实例化
        if (devices[d_idx] == nullptr) {
            int32_t handle = open_device_handles[global_motors[i].info.device_name];
            int type = global_motors[i].info.api_type; // 从 CSV 读取 API 类型
            std::cout << "[Init] Creating Device Driver for " << global_motors[i].info.device_name 
                      << " | API Type: " << type << std::endl;
            devices[d_idx] = new DeviceX(); // 原 RS 电机

            // 初始化设备线程
            devices[d_idx]->Init(handle, global_motors[i].info.device_name, d_idx, &global_motors, &mapper);
        }
        else {
            // 可选：检查同一张卡下的电机类型是否一致
            // 这是一个简单的校验，防止一张卡上混插了不同协议的电机（这通常是不允许的）
            // 如果你的硬件确实混插了，可以注释掉下面这段 Warning
            (void)global_motors[i].info.api_type;
            // 这里我们无法简单地从 BaseDevice* 反推类型，除非加虚函数 getTypeId，
            // 但作为初始化检查，我们假设第一个电机的类型决定了该设备的类型。
        }
    }
}


BaseRobot::~BaseRobot() { 
    for (auto d : devices) if(d) delete d; 
    for(auto const& [name, handle] : open_device_handles) {
        closeUSBCAN(handle);
    }
}





int BaseRobot::SetKpd_N(float kp, float kd, int N) {
    // 1. 检查索引合法性
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    // 2. 更新配置信息中的参数 (in_use 记录)
    global_motors[N].info.kp_in_use = (float)kp;
    global_motors[N].info.kd_in_use = (float)kd;

    // 3. 更新发送结构体中的参数 (实际发给电机的参数)
    // 这样在下次执行 move_p 或 SendCommand 时，会使用新的增益
    global_motors[N].send.kp = (float)kp;
    global_motors[N].send.kd = (float)kd;

    return 0;
}


void BaseRobot::SetKpd_all(float kp, float kd) {
    // 遍历所有电机并调用单轴设置函数
    for (int i = 0; i < (int)global_motors.size(); ++i) {
        SetKpd_N(kp, kd, i);
    }
    std::cout << "[Info] Set KP=" << kp << ", KD=" << kd << " for all motors." << std::endl;
}

int BaseRobot::SetType5CurrentPI_N(uint32_t kp, uint32_t ki, int N, bool save) {
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    if (global_motors[N].info.api_type != 5) {
        std::cerr << "[Error] Motor index " << N << " is not TYPE5." << std::endl;
        return -2;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (dev_idx < 0 || dev_idx >= (int)devices.size() || !devices[dev_idx]) {
        std::cerr << "[Error] Device not found for motor index: " << N << std::endl;
        return -3;
    }

    return devices[dev_idx]->SetType5CurrentPI(N, kp, ki, save);
}

int BaseRobot::SetType5SpeedPI_N(uint32_t kp, uint32_t ki, int N, bool save) {
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    if (global_motors[N].info.api_type != 5) {
        std::cerr << "[Error] Motor index " << N << " is not TYPE5." << std::endl;
        return -2;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (dev_idx < 0 || dev_idx >= (int)devices.size() || !devices[dev_idx]) {
        std::cerr << "[Error] Device not found for motor index: " << N << std::endl;
        return -3;
    }

    return devices[dev_idx]->SetType5SpeedPI(N, kp, ki, save);
}

int BaseRobot::SetType5PositionPI_N(uint32_t kp, uint32_t ki, int N, bool save) {
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    if (global_motors[N].info.api_type != 5) {
        std::cerr << "[Error] Motor index " << N << " is not TYPE5." << std::endl;
        return -2;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (dev_idx < 0 || dev_idx >= (int)devices.size() || !devices[dev_idx]) {
        std::cerr << "[Error] Device not found for motor index: " << N << std::endl;
        return -3;
    }

    return devices[dev_idx]->SetType5PositionPI(N, kp, ki, save);
}

int BaseRobot::SaveType5Params_N(int N) {
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    if (global_motors[N].info.api_type != 5) {
        std::cerr << "[Error] Motor index " << N << " is not TYPE5." << std::endl;
        return -2;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (dev_idx < 0 || dev_idx >= (int)devices.size() || !devices[dev_idx]) {
        std::cerr << "[Error] Device not found for motor index: " << N << std::endl;
        return -3;
    }

    return devices[dev_idx]->SaveType5Params(N);
}




void BaseRobot::EnableAll() { 
    for (int i = 0; i < global_motors.size(); i++) {
        int dev_idx = global_motors[i].info.device_index;
        if (devices[dev_idx]) {
            devices[dev_idx]->EnableMotor( i);
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
        }
    }
     }

void BaseRobot::DisableAll() { 
    for (int i = 0; i < global_motors.size(); i++) {
        int dev_idx = global_motors[i].info.device_index;
        if (devices[dev_idx]) {
            devices[dev_idx]->DisableMotor( i);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }
    }

}

void BaseRobot::Enable_N(int N) {
        int dev_idx = global_motors[N].info.device_index;
        if (devices[dev_idx]) {
            devices[dev_idx]->EnableMotor( N);
        }
        else std::cout <<"enable failed device"<<N<<" not found."<<std::endl;
}
void BaseRobot::Disable_N(int N) { 
        int dev_idx = global_motors[N].info.device_index;
        if (devices[dev_idx]) {
            devices[dev_idx]->DisableMotor( N);
        }
        else std::cout <<"device not found."<<std::endl;
    }

void BaseRobot::ClearError_N(int N) {
    // 1. 检查索引合法性
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return;
    }

    // 2. 获取对应的设备驱动索引
    int dev_idx = global_motors[N].info.device_index;

    // 3. 调用设备接口执行清除故障 (Type 4, Data[0]=1)
    if (devices[dev_idx]) {
        devices[dev_idx]->ClearError(N);
        std::cout << "[Info] Clear Error sent to Motor index: " << N 
                  << " (" << global_motors[N].info.name << ")" << std::endl;
    } else {
        std::cerr << "[Error] Device not found for motor index: " << N << std::endl;
    }
}

void BaseRobot::ClearError_All() {
    // 遍历所有电机并逐一清除错误
    for (int i = 0; i < (int)global_motors.size(); i++) {
        ClearError_N(i);
        
        // 适当延时防止 CAN 总线瞬时压力过大
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[Info] Clear Error command sent to all motors." << std::endl;
}


void BaseRobot::SetMode_N(int N,int mode){

        int dev_idx = global_motors[N].info.device_index;
        if (devices[dev_idx]) {
            devices[dev_idx]->SetMode(N, mode);
        }
        else std::cout <<"device not found."<<std::endl;
    }

/**
 * @brief 统一设置所有电机的模式
 * @param modes 输入的模式向量，对应标准：0-运控, 1-位置, 2-速度, 3-电流
 */
void BaseRobot::SetModes(std::vector<int>& modes) {
    size_t count = std::min(modes.size(), global_motors.size());
    for (size_t i = 0; i < count; ++i) {
        SetMode_N(static_cast<int>(i), modes[i]);
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    std::cout << "[Info] Batch SetModes completed for " << count << " motors." << std::endl;
}



void BaseRobot::SetModeAll_TypeX(int X, int mode) {
    // 遍历所有电机
    for (int i = 0; i < global_motors.size(); i++) {
        // 【关键】只处理 api_type 为 X 的电机 (通常是 RS/宇树协议)
        if (global_motors[i].info.api_type == X) {
            int dev_idx = global_motors[i].info.device_index;
            
            // 确保设备指针有效
            if (dev_idx >= 0 && dev_idx < devices.size() && devices[dev_idx]) {
                devices[dev_idx]->SetMode(i, mode);
                
                // 硬件通信通常需要一点延时防止总线拥堵
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
            }
        }
    }
}



// 在 .cpp 中实现:
void BaseRobot::Move_N(int N, const MotorCmdVec& target) {
    // 1. 边界检查 (保留，安全第一)
    if (N < 0 || N >= global_motors.size()) {
        // std::cerr 太慢，建议生产环境去掉打印或使用专门的日志库
        return; 
    }

    // [优化] 使用引用别名，告诉编译器"这一坨东西就是那个内存地址"
    // 避免后面反复写 global_motors[N] 导致多次计算偏移量
    auto& motor = global_motors[N];
    const float pos_clip_min = (motor.info.pos_max > motor.info.pos_min) ? motor.info.pos_min : motor.info.p_min;
    const float pos_clip_max = (motor.info.pos_max > motor.info.pos_min) ? motor.info.pos_max : motor.info.p_max;

    // 2. 限幅与赋值
    // 由于 Clip 是 inline 的，这里实际上没有函数调用，全是直接的数学指令
    motor.send.position = Clip(target.p, pos_clip_min, pos_clip_max);
    motor.send.speed    = target.v;
    motor.send.torque   = target.t;

    // 3. 发送指令
    int dev_idx = motor.info.device_index;
    
    // [优化] 这一行检查是指针检查，开销极小
    if (dev_idx >= 0 && dev_idx < devices.size() && devices[dev_idx]) {
        devices[dev_idx]->SendCommand(N); 
    }
}



void BaseRobot::Move(const std::vector<MotorCmdVec>& targets) {
    // 1. 缓存大小，避免每次循环都调用 .size()
    const int count = std::min(targets.size(), global_motors.size());
    for (int i = 0; i < count; i++) {
        // [优化] 这里的 const auto& 是必须的，零拷贝
        const auto& cmd = targets[i];
        
        // [优化] 获取当前电机对象的引用，后续所有操作都基于这个引用
        auto& motor = global_motors[i]; 
        const float pos_clip_min = (motor.info.pos_max > motor.info.pos_min) ? motor.info.pos_min : motor.info.p_min;
        const float pos_clip_max = (motor.info.pos_max > motor.info.pos_min) ? motor.info.pos_max : motor.info.p_max;

        // 2. 限幅与赋值 (内联展开，极速)
        motor.send.position = Clip(cmd.p, pos_clip_min, pos_clip_max); 
        motor.send.speed    = cmd.v;
        motor.send.torque   = cmd.t;
        // 3. 发送逻辑
        int dev_idx = motor.info.device_index;
        // 通常 devices 不会为空，如果为了极限性能且确保初始化无误，可以去掉 if (devices[dev_idx]) 检查
        // 但保留它更安全，开销在纳秒级
        if (devices[dev_idx]) {
            // static_cast 编译期完成，无运行时开销
            devices[dev_idx]->SendCommand(i);
        }
        // [性能瓶颈提示] 
        // 如果你的 CAN 总线带宽允许，或者底层 buffer 够大，建议减小或移除这个延时。
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}



void BaseRobot::PrintStatus() {
    std::cout << "\n--- Robot Status (10Hz) | Total Motors: " << global_motors.size() << " ---\n";
    for (int i = 0; i < global_motors.size(); ++i) {
        const auto& m = global_motors[i];
        if (m.info.name.empty()) continue;
        float rx_pos = m.recv.current_position_f.load();
        float rx_vel = m.recv.current_speed_f.load();
        int fault_message = m.recv.fault_message;
        std::cout << "Idx: " << i << " | ID: " << m.info.num << " [" << m.info.name << "] "
                  << " | Cmd: " << std::fixed << std::setprecision(2) << m.send.position
                  << "fault mesage"<< fault_message
                  << " | ReadPos: " << rx_pos << " | ReadVel: " << rx_vel << std::endl;


    }
    std::cout << "--------------------\n";
}
// 获取所有电机的当前位置
std::vector<float> BaseRobot::GetPosAll() {
    std::vector<float> positions;
    positions.reserve(global_motors.size()); // 预分配内存提高性能

    for (const auto& m : global_motors) {
        // 使用 .load() 确保从 atomic 变量中安全读取
        positions.push_back(m.recv.current_position_f.load());
    }
    return positions;
}

// 获取前 N 个电机的当前位置
std::vector<float> BaseRobot::GetPosN(int n) {
    std::vector<float> positions;
    
    // 确定读取边界，防止索引越界
    int count = std::min(n, static_cast<int>(global_motors.size()));
    if (count <= 0) return positions;

    positions.reserve(count);
    for (int i = 0; i < count; ++i) {
        positions.push_back(global_motors[i].recv.current_position_f.load());
    }
    return positions;
}

// =========================================================
//  Query Position Implementation (新增接口实现)
// =========================================================

void BaseRobot::QueryPos_N(int N) {
    // 1. 边界检查
    if (N < 0 || N >= global_motors.size()) {
        std::cerr << "[Error] QueryPos_N: Motor index " << N << " out of range!" << std::endl;
        return;
    }

    // 2. 获取设备索引
    int dev_idx = global_motors[N].info.device_index;

    // 3. 调用 DeviceX 的 QueryPos 接口
    // 确保设备指针有效
    if (dev_idx >= 0 && dev_idx < devices.size() && devices[dev_idx]) {
        devices[dev_idx]->QueryPos(N);
    } else {
        std::cerr << "[Error] QueryPos_N: Device not found for motor " << N << std::endl;
    }
}

void BaseRobot::QueryPos_ALL() {
    // 遍历所有电机
    for (int i = 0; i < global_motors.size(); i++) {
        int dev_idx = global_motors[i].info.device_index;
        
        // 确保设备指针有效
        if (dev_idx >= 0 && dev_idx < devices.size() && devices[dev_idx]) {
            // 发送查询指令
            devices[dev_idx]->QueryPos(i);
            
            // 【重要】CAN总线流控延时
            // 虽然查询指令很短，但如果32个电机连续发送，可能会填满CAN卡缓冲区
            // 这里给予 200微秒 的间隔
            std::this_thread::sleep_for(std::chrono::microseconds(200)); 
        }
    }
    // 可选：打印日志
    // std::cout << "[Info] Query Position command sent to all motors." << std::endl;
}
