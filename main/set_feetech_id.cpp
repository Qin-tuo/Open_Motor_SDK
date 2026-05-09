#include "feetech_servo_device.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void print_usage() {
    std::cout << "Usage: ros2 run khcan set_feetech_id "
              << "--port /dev/ttyACM0 --old-id 1 --new-id 2 [--baud 500000]\n"
              << "\n"
              << "Safety: connect only one Feetech servo while changing ID." << std::endl;
}

bool parse_int(const std::string& value, int& out) {
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0') {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

bool valid_servo_id(int id) {
    return id >= 0 && id <= 253;
}

}  // namespace

int main(int argc, char** argv) {
    std::string port;
    int baud = 500000;
    int old_id = -1;
    int new_id = -1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[Error] Missing value for " << name << std::endl;
                print_usage();
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg == "--port") {
            port = require_value("--port");
        } else if (arg == "--baud") {
            if (!parse_int(require_value("--baud"), baud)) {
                std::cerr << "[Error] Invalid --baud." << std::endl;
                return 1;
            }
        } else if (arg == "--old-id") {
            if (!parse_int(require_value("--old-id"), old_id)) {
                std::cerr << "[Error] Invalid --old-id." << std::endl;
                return 1;
            }
        } else if (arg == "--new-id") {
            if (!parse_int(require_value("--new-id"), new_id)) {
                std::cerr << "[Error] Invalid --new-id." << std::endl;
                return 1;
            }
        } else if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        } else {
            std::cerr << "[Error] Unknown argument: " << arg << std::endl;
            print_usage();
            return 1;
        }
    }

    if (port.empty() || !valid_servo_id(old_id) || !valid_servo_id(new_id)) {
        std::cerr << "[Error] Required: --port, --old-id 0~253, --new-id 0~253." << std::endl;
        print_usage();
        return 1;
    }

    if (old_id == new_id) {
        std::cerr << "[Error] old-id and new-id are the same." << std::endl;
        return 1;
    }

    std::cout << "[Warn] Connect only ONE Feetech servo before changing ID." << std::endl;
    std::cout << "[Info] Changing Feetech ID on " << port
              << " @ " << baud
              << ": " << old_id << " -> " << new_id << std::endl;

    FeetechServoDevice device;
    if (!device.Init(port, baud, 0, nullptr, nullptr)) {
        return 1;
    }

    if (!device.SetServoId(old_id, new_id)) {
        std::cerr << "[Error] Failed to change Feetech servo ID." << std::endl;
        return 1;
    }

    std::cout << "[Info] Feetech ID write command sent successfully." << std::endl;
    std::cout << "[Info] Power-cycle the servo, then update motor.toml canid = "
              << new_id << "." << std::endl;
    return 0;
}
