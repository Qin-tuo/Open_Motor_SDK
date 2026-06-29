#include "robot.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
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

        const std::string config = get_parameter("config").as_string();
        const double rate_hz = std::max(1.0, get_parameter("rate_hz").as_double());
        robot_ = std::make_unique<BaseRobot>(config);
        build_indexes();
        create_channel_interfaces();

        const int default_mode = static_cast<int>(get_parameter("default_mode").as_int());
        if (default_mode >= 0) {
            for (int i = 0; i < static_cast<int>(robot_->global_motors.size()); ++i) {
                robot_->SetMode_N(i, default_mode);
            }
        }

        if (get_parameter("auto_enable").as_bool()) {
            robot_->ClearError_All();
            robot_->EnableAll();
        }

        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&MotorDriverNode::tick, this));

        RCLCPP_INFO(get_logger(), "motor driver node ready: motors=%zu channels=%zu",
                    robot_->global_motors.size(), channels_.size());
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
        if (!robot_) return;

        for (int i = 0; i < static_cast<int>(robot_->global_motors.size()); ++i) {
            const auto& info = robot_->global_motors[static_cast<std::size_t>(i)].info;

            auto& channel = channels_[info.chan];
            channel.chan = info.chan;
            channel.motor_indices.push_back(i);
            channel.name_to_index[info.name] = i;
        }
    }

    void create_channel_interfaces() {
        for (auto& entry : channels_) {
            auto& channel = entry.second;
            const std::string prefix = "/can" + std::to_string(channel.chan);

            channel.command_sub = create_subscription<sensor_msgs::msg::JointState>(
                prefix + "/command", 10,
                [this, chan = channel.chan](sensor_msgs::msg::JointState::SharedPtr msg) {
                    on_command(chan, std::move(msg));
                });
            channel.state_pub = create_publisher<sensor_msgs::msg::JointState>(
                prefix + "/joint_states", 10);
            channel.status_pub = create_publisher<khcan::msg::MotorStatusArray>(
                prefix + "/status_feedback", 10);
            channel.enable_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/enable",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    (void)req;
                    for_channel(chan, [this](int idx) { robot_->Enable_N(idx); });
                    res->success = true;
                    res->message = "enabled";
                });
            channel.disable_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/disable",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    (void)req;
                    for_channel(chan, [this](int idx) { robot_->Disable_N(idx); });
                    res->success = true;
                    res->message = "disabled";
                });
            channel.clear_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/clear_error",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    (void)req;
                    for_channel(chan, [this](int idx) { robot_->ClearError_N(idx); });
                    res->success = true;
                    res->message = "clear_error sent";
                });
            channel.set_zero_srv = create_service<std_srvs::srv::Trigger>(
                prefix + "/set_zero",
                [this, chan = channel.chan](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    (void)req;
                    for_channel(chan, [this](int idx) { robot_->SetZero_N(idx); });
                    res->success = true;
                    res->message = "set_zero sent";
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
    void for_channel(int chan, Func func) {
        auto it = channels_.find(chan);
        if (it == channels_.end()) return;
        for (int idx : it->second.motor_indices) {
            func(idx);
        }
    }

    void on_command(int chan, sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!robot_) return;
        auto it = channels_.find(chan);
        if (it == channels_.end()) return;
        auto& channel = it->second;

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
                if (stage_one(name_it->second, *msg, i)) {
                    staged.push_back(name_it->second);
                }
            }
        } else {
            const std::size_t requested_count = std::max({
                msg->position.size(),
                msg->velocity.size(),
                msg->effort.size(),
            });
            const std::size_t count = std::min(requested_count, channel.motor_indices.size());
            for (std::size_t i = 0; i < count; ++i) {
                const int motor_index = channel.motor_indices[i];
                if (stage_one(motor_index, *msg, i)) {
                    staged.push_back(motor_index);
                }
            }
        }

        for (int motor_index : staged) {
            robot_->Flush_N(motor_index);
        }
    }

    bool stage_one(int motor_index, const sensor_msgs::msg::JointState& msg, std::size_t src_index) {
        if (motor_index < 0 ||
            static_cast<std::size_t>(motor_index) >= robot_->global_motors.size()) {
            return false;
        }

        const auto& motor = robot_->global_motors[static_cast<std::size_t>(motor_index)];
        MotorCmdVec cmd {};
        cmd.p = (src_index < msg.position.size())
            ? static_cast<float>(msg.position[src_index])
            : motor.send.position;
        cmd.v = (src_index < msg.velocity.size())
            ? static_cast<float>(msg.velocity[src_index])
            : motor.send.speed;
        cmd.t = (src_index < msg.effort.size())
            ? static_cast<float>(msg.effort[src_index])
            : motor.send.torque;
        return robot_->Stage_N(motor_index, cmd);
    }

    void tick() {
        if (!robot_) return;
        robot_->QueryPos_ALL();
        for (auto& entry : channels_) {
            publish_channel_state(entry.second);
        }
    }

    void publish_channel_state(ChannelContext& channel) {
        const auto stamp = now();

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        msg.name.reserve(channel.motor_indices.size());
        msg.position.reserve(channel.motor_indices.size());
        msg.velocity.reserve(channel.motor_indices.size());
        msg.effort.reserve(channel.motor_indices.size());

        khcan::msg::MotorStatusArray status;
        status.header.stamp = stamp;
        status.motors.reserve(channel.motor_indices.size());

        for (int idx : channel.motor_indices) {
            const auto& motor = robot_->global_motors[static_cast<std::size_t>(idx)];
            const auto& recv = motor.recv;

            msg.name.push_back(motor.info.name);
            msg.position.push_back(recv.current_position_f.load());
            msg.velocity.push_back(recv.current_speed_f.load());
            msg.effort.push_back(recv.current_torque_f.load());

            khcan::msg::MotorStatus s;
            s.name = motor.info.name;
            s.motor_id = recv.motor_id;
            s.mode = recv.mode;
            s.motor_state = recv.motor_state;
            s.fault_code = recv.fault_message.load();
            s.position = recv.current_position_f.load();
            s.speed = recv.current_speed_f.load();
            s.torque = recv.current_torque_f.load();
            s.current = recv.current_iq_f.load();
            s.temperature = recv.current_temp_f.load();
            status.motors.push_back(std::move(s));
        }
        channel.state_pub->publish(msg);
        channel.status_pub->publish(status);
    }

    void on_set_mode(int chan,
                     const std::shared_ptr<khcan::srv::SetMotorMode::Request> req,
                     std::shared_ptr<khcan::srv::SetMotorMode::Response> res) {
        auto it = channels_.find(chan);
        if (it == channels_.end()) {
            res->success = false;
            res->message = "channel not found";
            return;
        }

        std::vector<std::string> unknown;
        int changed = 0;
        if (req->names.empty()) {
            for (int idx : it->second.motor_indices) {
                if (robot_->SetMode_N(idx, req->mode)) {
                    ++changed;
                }
            }
        } else {
            for (const auto& name : req->names) {
                const auto name_it = it->second.name_to_index.find(name);
                if (name_it == it->second.name_to_index.end()) {
                    unknown.push_back(name);
                    continue;
                }
                if (robot_->SetMode_N(name_it->second, req->mode)) {
                    ++changed;
                }
            }
        }

        if (!unknown.empty()) {
            std::string message = "unknown motors:";
            for (const auto& name : unknown) {
                message += " " + name;
            }
            if (changed > 0) {
                message += "; changed " + std::to_string(changed) + " motors";
            }
            res->success = false;
            res->message = message;
            return;
        }

        res->success = true;
        res->message = "mode " + std::to_string(req->mode) + " set for " +
                       std::to_string(changed) + " motors";
    }

    std::unique_ptr<BaseRobot> robot_;
    std::map<int, ChannelContext> channels_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriverNode>());
    rclcpp::shutdown();
    return 0;
}
