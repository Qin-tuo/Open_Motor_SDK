#include "robot.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::atomic<bool> running{true};

void handleSignal(int) {
    running = false;
}

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options] [motor.toml]\n"
        << "\n"
        << "Default mode is status query only: no disable, no clear-error, no enable.\n"
        << "\n"
        << "Options:\n"
        << "  --resolve-config-only   Print resolved config path and exit without opening CAN\n"
        << "  --hz <value>            Query/print rate, default 10\n"
        << "  --clear-errors          Send clear-error before monitoring\n"
        << "  --enable                Send enable before monitoring\n"
        << "  --disable-first         Send disable before monitoring\n"
        << "  --disable-on-exit       Send disable when exiting\n"
        << "  --help                  Show this message\n";
}

std::string defaultConfigPath() {
    if (const char* env_path = std::getenv("KHCAN_CONFIG_PATH")) {
        if (env_path[0] != '\0') {
            return env_path;
        }
    }

    try {
        const std::string package_share_directory =
            ament_index_cpp::get_package_share_directory("khcan");
        return package_share_directory + "/config/motor.toml";
    } catch (const std::exception&) {
        const std::filesystem::path local_motor_driver =
            "src/motor_driver/config/motor.toml";
        if (std::filesystem::exists(local_motor_driver)) {
            return local_motor_driver.string();
        }
        const std::filesystem::path local_khcan = "src/khcan/config/motor.toml";
        if (std::filesystem::exists(local_khcan)) {
            return local_khcan.string();
        }
        throw;
    }
}

double parsePositiveDouble(const std::string& value, const std::string& option_name) {
    const double parsed = std::stod(value);
    if (!(parsed > 0.0)) {
        throw std::invalid_argument(option_name + " must be > 0");
    }
    return parsed;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    bool resolve_config_only = false;
    bool clear_errors = false;
    bool enable = false;
    bool disable_first = false;
    bool disable_on_exit = false;
    double target_hz = 10.0;
    std::string config_path;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            }
            if (arg == "--resolve-config-only") {
                resolve_config_only = true;
                continue;
            }
            if (arg == "--clear-errors") {
                clear_errors = true;
                continue;
            }
            if (arg == "--enable") {
                enable = true;
                continue;
            }
            if (arg == "--disable-first") {
                disable_first = true;
                continue;
            }
            if (arg == "--disable-on-exit") {
                disable_on_exit = true;
                continue;
            }
            if (arg == "--hz") {
                if (i + 1 >= argc) {
                    throw std::invalid_argument("--hz requires a value");
                }
                target_hz = parsePositiveDouble(argv[++i], "--hz");
                continue;
            }
            if (!arg.empty() && arg[0] == '-') {
                throw std::invalid_argument("unknown option: " + arg);
            }
            if (!config_path.empty()) {
                throw std::invalid_argument("multiple config paths were provided");
            }
            config_path = arg;
        }

        if (config_path.empty()) {
            config_path = defaultConfigPath();
        }
    } catch (const std::exception& ex) {
        std::cerr << "[show_status] argument error: " << ex.what() << std::endl;
        printUsage(argv[0]);
        return 2;
    }

    std::cout << "[show_status] config: " << config_path << std::endl;
    if (resolve_config_only) {
        return 0;
    }

    try {
    BaseRobot robot(config_path, disable_on_exit);

    if (disable_first) {
        if (!robot.DisableAll()) throw std::runtime_error("disable-first failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (clear_errors) {
        if (!robot.ClearError_All()) throw std::runtime_error("clear-errors failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (enable) {
        if (!robot.EnableAll()) throw std::runtime_error("enable failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const auto period_duration =
        std::chrono::microseconds(static_cast<int64_t>(1000000.0 / target_hz));

    std::cout << "=== Robot Position Monitor (" << target_hz << "Hz) ===" << std::endl;
    std::cout << "query-only by default; Press Ctrl+C to exit." << std::endl;

    while (running.load()) {
        const auto start_time = std::chrono::steady_clock::now();

        robot.QueryPos_ALL();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        robot.PrintStatus();

        const auto end_time = std::chrono::steady_clock::now();
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        if (elapsed < period_duration) {
            std::this_thread::sleep_for(period_duration - elapsed);
        }
    }

    return 0;
    } catch (const std::exception& ex) {
        std::cerr << "[show_status] " << ex.what() << std::endl;
        return 1;
    }
}
