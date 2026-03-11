#include "robot.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

BaseRobot::BaseRobot(const std::string& config_file) {
    auto infos = MotorConfigLoader::loadConfig(config_file);
    if (infos.empty()) { std::cerr << "Error: No motors.\n"; return; }

    int max_canid = 0;
    int device_counter = 0;

    // 1. 建立设备名到索引的映�?
    for (auto& info : infos) {
        if (device_name_map_idx.find(info.device_name) == device_name_map_idx.end()) {
            device_name_map_idx[info.device_name] = device_counter++;
        }
        if (info.canid > max_canid) max_canid = info.canid;
    }

    // 2. 初始化全局电机向量
    global_motors.resize(infos.size()); 
    std::vector<std::vector<uint>> topo_matrix;
    
    for (int i = 0; i < infos.size(); ++i) {
        auto& info = infos[i];
        
        info.device_index = device_name_map_idx[info.device_name];

        global_motors[i].info = info;
        // �?CSV 里的默认 kp/kd 填入发送结构体作为初始�?
        global_motors[i].send.kd = info.kd_in_use;
        global_motors[i].send.kp = info.kp_in_use;

        topo_matrix.push_back({
            (uint)info.device_index, 
            0u, 
            (uint)info.canid, 
            (uint)i
        });
    }

    // 3. 初始化拓扑映射器
    mapper = TopoMapper({(uint)device_counter, 1u, (uint)max_canid + 1}, topo_matrix);

    devices.resize(device_counter, nullptr);
    // 4. Create socketcan devices

    // 5. 根据 api_type 实例化对应的设备驱动�?(Type1 or Type2)
    for (int i = 0; i < global_motors.size(); ++i) {
        int d_idx = global_motors[i].info.device_index;
        
        // 如果该设�?SocketCAN�?还未被实例化
        if (devices[d_idx] == nullptr) {
                        int type = global_motors[i].info.api_type; // �?CSV 读取 API 类型
            std::cout << "[Init] Creating Device Driver for " << global_motors[i].info.device_name 
                      << " | API Type: " << type << std::endl;
            devices[d_idx] = new DeviceX(); // �?RS 电机

            // 初始化设备线�?
            devices[d_idx]->Init(global_motors[i].info.device_name, d_idx, &global_motors, &mapper);
        }
        else {
            // 可选：检查同一张卡下的电机类型是否一�?
            // 这是一个简单的校验，防止一张卡上混插了不同协议的电机（这通常是不允许的）
            // 如果你的硬件确实混插了，可以注释掉下面这�?Warning
            int current_motor_type = global_motors[i].info.api_type;
            // 这里我们无法简单地�?BaseDevice* 反推类型，除非加虚函�?getTypeId�?
            // 但作为初始化检查，我们假设第一个电机的类型决定了该设备的类型�?
        }
    }
}


BaseRobot::~BaseRobot() {
    try {
        DisableAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (...) {
    }

    for (auto d : devices) if(d) delete d;
}





int BaseRobot::SetKpd_N(float kp, float kd, int N) {
    // 1. 检查索引合法�?
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    // 2. 更新配置信息中的参数 (in_use 记录)
    global_motors[N].info.kp_in_use = (float)kp;
    global_motors[N].info.kd_in_use = (float)kd;

    // 3. 更新发送结构体中的参数 (实际发给电机的参�?
    // 这样在下次执�?move_p �?SendCommand 时，会使用新的增�?
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
    // 1. 检查索引合法�?
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return;
    }

    // 2. 获取对应的设备驱动索�?
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

void BaseRobot::SetZero_N(int N) {
    if (N < 0 || N >= (int)global_motors.size()) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return;
    }

    int dev_idx = global_motors[N].info.device_index;
    if (devices[dev_idx]) {
        devices[dev_idx]->SetZero(N);
        std::cout << "[Info] Set Zero sent to Motor index: " << N
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

void BaseRobot::SetZero_All() {
    for (int i = 0; i < (int)global_motors.size(); i++) {
        SetZero_N(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[Info] Set Zero command sent to all motors." << std::endl;
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
 * @param modes 输入的模式向量，对应标准�?-运控, 1-位置, 2-速度, 3-电流
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
    // 遍历所有电�?
    for (int i = 0; i < global_motors.size(); i++) {
        // 【关键】只处理 api_type �?X 的电�?(通常�?RS/宇树协议)
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



// �?.cpp 中实�?
void BaseRobot::Move_N(int N, const MotorCmdVec& target) {
    // 1. 边界检�?(保留，安全第一)
    if (N < 0 || N >= global_motors.size()) {
        // std::cerr 太慢，建议生产环境去掉打印或使用专门的日志库
        return; 
    }

    // [优化] 使用引用别名，告诉编译器"这一坨东西就是那个内存地址"
    // 避免后面反复�?global_motors[N] 导致多次计算偏移�?
    auto& motor = global_motors[N];

    // 2. 限幅与赋�?
    // 由于 Clip �?inline 的，这里实际上没有函数调用，全是直接的数学指�?
    motor.send.position = Clip(target.p, motor.info.pos_min, motor.info.pos_max);
    motor.send.speed    = target.v;
    motor.send.torque   = target.t;

    // 3. 发送指�?
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
        // [优化] 这里�?const auto& 是必须的，零拷贝
        const auto& cmd = targets[i];
        
        // [优化] 获取当前电机对象的引用，后续所有操作都基于这个引用
        auto& motor = global_motors[i]; 

        // 2. 限幅与赋�?(内联展开，极�?
        motor.send.position = Clip(cmd.p, motor.info.pos_min, motor.info.pos_max); 
        motor.send.speed    = cmd.v;
        motor.send.torque   = cmd.t;
        // 3. 发送逻辑
        int dev_idx = motor.info.device_index;
        // 通常 devices 不会为空，如果为了极限性能且确保初始化无误，可以去�?if (devices[dev_idx]) 检�?
        // 但保留它更安全，开销在纳秒级
        if (devices[dev_idx]) {
            // static_cast 编译期完成，无运行时开销
            devices[dev_idx]->SendCommand(i);
        }
        // [性能瓶颈提示] 
        // 如果你的 CAN 总线带宽允许，或者底�?buffer 够大，建议减小或移除这个延时�?
        std::this_thread::sleep_for(std::chrono::microseconds(100));
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
        // 使用 .load() 确保�?atomic 变量中安全读�?
        positions.push_back(m.recv.current_position_f.load());
    }
    return positions;
}

// 获取�?N 个电机的当前位置
std::vector<float> BaseRobot::GetPosN(int n) {
    std::vector<float> positions;
    
    // 确定读取边界，防止索引越�?
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
    // 1. 边界检�?
    if (N < 0 || N >= global_motors.size()) {
        std::cerr << "[Error] QueryPos_N: Motor index " << N << " out of range!" << std::endl;
        return;
    }

    // 2. 获取设备索引
    int dev_idx = global_motors[N].info.device_index;

    // 3. 调用 DeviceX �?QueryPos 接口
    // 确保设备指针有效
    if (dev_idx >= 0 && dev_idx < devices.size() && devices[dev_idx]) {
        devices[dev_idx]->QueryPos(N);
    } else {
        std::cerr << "[Error] QueryPos_N: Device not found for motor " << N << std::endl;
    }
}

void BaseRobot::QueryPos_ALL() {
    // 遍历所有电�?
    for (int i = 0; i < global_motors.size(); i++) {
        int dev_idx = global_motors[i].info.device_index;
        
        // 确保设备指针有效
        if (dev_idx >= 0 && dev_idx < devices.size() && devices[dev_idx]) {
            // 发送查询指�?
            devices[dev_idx]->QueryPos(i);
            
            // 【重要】CAN总线流控延时
            // 虽然查询指令很短，但如果32个电机连续发送，可能会填满CAN卡缓冲区
            // 这里给予 200微秒 的间�?
            std::this_thread::sleep_for(std::chrono::microseconds(200)); 
        }
    }
    // 可选：打印日志
    // std::cout << "[Info] Query Position command sent to all motors." << std::endl;
}
