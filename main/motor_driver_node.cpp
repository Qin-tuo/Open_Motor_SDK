#include "robot.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "khcan/msg/motor_status_array.hpp"
#include "khcan/srv/set_motor_mode.hpp"

class MotorDriverNode : public rclcpp::Node {
public:
    MotorDriverNode() : Node("motor_driver_node") {
        const std::string default_config =
            ament_index_cpp::get_package_share_directory("khcan") + "/config/motor.toml";
        declare_parameter<std::string>("config", default_config);
        declare_parameter<double>("rate_hz", 100.0);
        declare_parameter<bool>("auto_enable", false);
        declare_parameter<int>("default_mode", -1);
        declare_parameter<int>("command_timeout_ms", 100);
        declare_parameter<int>("feedback_timeout_ms", 500);

        const std::string config = get_parameter("config").as_string();
        const double rate_hz = std::max(1.0, get_parameter("rate_hz").as_double());
        feedback_timeout_ms_ = static_cast<uint64_t>(
            std::max<int64_t>(1, get_parameter("feedback_timeout_ms").as_int()));

        robot_ = std::make_unique<BaseRobot>(config);
        robot_->SetCommandTimeout(std::chrono::milliseconds(
            std::max<int64_t>(0, get_parameter("command_timeout_ms").as_int())));
        build_indexes();
        create_channel_interfaces();

        const int default_mode = static_cast<int>(get_parameter("default_mode").as_int());
        if (default_mode >= 0) {
            for (int i = 0; i < static_cast<int>(robot_->MotorCount()); ++i) {
                if (!robot_->SetMode_N(i, default_mode)) {
                    throw std::runtime_error("Failed to set default motor mode");
                }
            }
        }

        if (get_parameter("auto_enable").as_bool()) {
            if (!robot_->EnableAll()) {
                throw std::runtime_error("Failed to auto-enable all motors");
            }
        }

        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MotorDriverNode::tick, this));

        RCLCPP_INFO(get_logger(), "motor driver node ready: motors=%zu channels=%zu",
                    robot_->MotorCount(), channels_.size());
    }

private:
    struct ChannelContext {
        int chan = -1;
        std::vector<int> motor_indices;
        std::unordered_map<std::string, int> name_to_index;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
        rclcpp::Publisher<khcan::msg::MotorStatusArray>::SharedPtr status_pub;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_zero_srv;
        rclcpp::Service<khcan::srv::SetMotorMode>::SharedPtr set_mode_srv;
    };

    void build_indexes() {
        channels_.clear();
        for (int i = 0; i < static_cast<int>(robot_->MotorCount()); ++i) {
            Motor_CAN_Info_Struct info;
            if (!robot_->GetMotorInfo(i, info)) {
                throw std::runtime_error("Failed to read motor configuration");
            }
            auto& channel = channels_[info.chan];
            channel.chan = info.chan;
            channel.motor_indices.push_back(i);
            channel.name_to_index.emplace(info.name, i);
        }
    }

    void create_channel_interfaces() {
        const rclcpp::QoS command_qos(rclcpp::KeepLast(1));
        for (auto& entry : channels_) {
            auto& channel = entry.second;
            const std::string prefix = "/can" + std::to_string(channel.chan);

            channel.command_sub = create_subscription<sensor_msgs::msg::JointState>(
                prefix + "/command", command_qos,
                [this, chan = channel.chan](sensor_msgs::msg::JointState::SharedPtr msg) {
                    on_command(chan, std::move(msg));
                });
            channel.state_pub = create_publisher<sensor_msgs::msg::JointState>(
                prefix + "/joint_states", rclcpp::QoS(10));
            channel.status_pub = create_publisher<khcan::msg::MotorStatusArray>(
                prefix + "/status_feedback", rclcpp::QoS(10));
            channel.enable_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/enable",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    res->success = for_channel(chan, [this](int idx) { return robot_->Enable_N(idx); });
                    res->message = res->success ? "enabled" : "one or more enable commands failed";
                });
            channel.disable_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/disable",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    res->success = for_channel(chan, [this](int idx) { return robot_->Disable_N(idx); });
                    res->message = res->success ? "disabled" : "one or more disable commands failed";
                });
            channel.clear_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/clear_error",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    res->success = for_channel(chan, [this](int idx) { return robot_->ClearError_N(idx); });
                    res->message = res->success ? "clear_error sent" : "one or more clear commands failed";
                });
            channel.set_zero_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/set_zero",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    res->success = for_channel(chan, [this](int idx) { return robot_->SetZero_N(idx); });
                    res->message = res->success ? "set_zero sent" : "one or more zero commands failed";
                });
            channel.set_mode_srv = create_service<khcan::srv::SetMotorMode>(
                prefix + "/set_mode",
                [this, chan = channel.chan](const std::shared_ptr<khcan::srv::SetMotorMode::Request> req,
                                            std::shared_ptr<khcan::srv::SetMotorMode::Response> res) {
                    on_set_mode(chan, req, res);
                });
        }
    }

    template <typename Func>
    bool for_channel(int chan, Func func) {
        const auto it = channels_.find(chan);
        if (it == channels_.end()) return false;
        bool ok = true;
        for (int idx : it->second.motor_indices) ok = func(idx) && ok;
        return ok;
    }

    void on_command(int chan, sensor_msgs::msg::JointState::SharedPtr msg) {
        const auto it = channels_.find(chan);
        if (it == channels_.end()) return;
        const auto& channel = it->second;

        std::vector<int> staged;
        if (!msg->name.empty()) {
            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                const auto name_it = channel.name_to_index.find(msg->name[i]);
                if (name_it == channel.name_to_index.end()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "unknown joint '%s' on can%d",
                                         msg->name[i].c_str(), chan);
                    continue;
                }
                if (stage_one(name_it->second, *msg, i)) staged.push_back(name_it->second);
            }
        } else {
            const std::size_t requested_count = std::max({
                msg->position.size(), msg->velocity.size(), msg->effort.size()});
            const std::size_t count = std::min(requested_count, channel.motor_indices.size());
            for (std::size_t i = 0; i < count; ++i) {
                if (stage_one(channel.motor_indices[i], *msg, i)) {
                    staged.push_back(channel.motor_indices[i]);
                }
            }
        }

        for (int motor_index : staged) {
            if (!robot_->Flush_N(motor_index)) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                      "command send failed on can%d", chan);
            }
        }
    }

    bool stage_one(int motor_index, const sensor_msgs::msg::JointState& msg,
                   std::size_t source_index) {
        if (source_index >= msg.position.size() && source_index >= msg.velocity.size() &&
            source_index >= msg.effort.size()) {
            return false;
        }
        MotorCmdVec command;
        if (!robot_->GetCommand_N(motor_index, command)) return false;
        if (source_index < msg.position.size()) command.p = static_cast<float>(msg.position[source_index]);
        if (source_index < msg.velocity.size()) command.v = static_cast<float>(msg.velocity[source_index]);
        if (source_index < msg.effort.size()) command.t = static_cast<float>(msg.effort[source_index]);
        return robot_->Stage_N(motor_index, command);
    }

    void tick() {
        const int expired = robot_->CheckCommandTimeouts();
        if (expired > 0) {
            RCLCPP_WARN(get_logger(), "command watchdog disabled %d motor(s)", expired);
        }
        if (!robot_->QueryPos_ALL()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "one or more feedback queries failed");
        }
        for (auto& entry : channels_) publish_channel_state(entry.second);
    }

    void publish_channel_state(ChannelContext& channel) {
        const auto stamp = now();
        const uint64_t now_ns = steady_time_ns();

        sensor_msgs::msg::JointState state;
        state.header.stamp = stamp;
        khcan::msg::MotorStatusArray status;
        status.header.stamp = stamp;

        for (int index : channel.motor_indices) {
            Motor_CAN_Struct motor;
            if (!robot_->GetMotorSnapshot(index, motor)) continue;
            const auto& recv = motor.recv;

            state.name.push_back(motor.info.name);
            state.position.push_back(recv.current_position_f.load());
            state.velocity.push_back(recv.current_speed_f.load());
            state.effort.push_back(recv.current_torque_f.load());

            const uint64_t age_ms = recv.last_feedback_ns == 0
                ? std::numeric_limits<uint32_t>::max()
                : (now_ns - recv.last_feedback_ns) / 1000000ULL;
            khcan::msg::MotorStatus item;
            item.name = motor.info.name;
            item.motor_id = recv.motor_id;
            item.mode = recv.mode;
            item.motor_state = recv.motor_state;
            item.fault_code = recv.fault_message.load();
            item.online = recv.last_feedback_ns != 0 && age_ms <= feedback_timeout_ms_;
            item.feedback_age_ms = static_cast<uint32_t>(
                std::min<uint64_t>(age_ms, std::numeric_limits<uint32_t>::max()));
            item.position = recv.current_position_f.load();
            item.speed = recv.current_speed_f.load();
            item.torque = recv.current_torque_f.load();
            item.current = recv.current_iq_f.load();
            item.temperature = recv.current_temp_f.load();
            status.motors.push_back(std::move(item));
        }
        channel.state_pub->publish(state);
        channel.status_pub->publish(status);
    }

    void on_set_mode(int chan,
                     const std::shared_ptr<khcan::srv::SetMotorMode::Request> req,
                     std::shared_ptr<khcan::srv::SetMotorMode::Response> res) {
        const auto it = channels_.find(chan);
        if (it == channels_.end()) {
            res->success = false;
            res->message = "channel not found";
            return;
        }

        bool ok = true;
        int changed = 0;
        std::vector<std::string> unknown;
        if (req->names.empty()) {
            for (int idx : it->second.motor_indices) {
                const bool changed_ok = robot_->SetMode_N(idx, req->mode);
                ok = changed_ok && ok;
                if (changed_ok) ++changed;
            }
        } else {
            for (const auto& name : req->names) {
                const auto name_it = it->second.name_to_index.find(name);
                if (name_it == it->second.name_to_index.end()) {
                    unknown.push_back(name);
                    ok = false;
                    continue;
                }
                const bool changed_ok = robot_->SetMode_N(name_it->second, req->mode);
                ok = changed_ok && ok;
                if (changed_ok) ++changed;
            }
        }

        res->success = ok;
        res->message = "mode " + std::to_string(req->mode) + " set for " +
                       std::to_string(changed) + " motors";
        if (!unknown.empty()) {
            res->message += "; unknown:";
            for (const auto& name : unknown) res->message += " " + name;
        } else if (!ok) {
            res->message += "; one or more commands failed";
        }
    }

    std::unique_ptr<BaseRobot> robot_;
    std::map<int, ChannelContext> channels_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t feedback_timeout_ms_ = 500;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MotorDriverNode>());
    } catch (const std::exception& error) {
        std::fprintf(stderr, "motor_driver_node: %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
