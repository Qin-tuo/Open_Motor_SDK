#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from khcan.srv import SetMotorMode


class NeckZeroEyeSwing(Node):
    def __init__(self):
        super().__init__("neck_zero_eye_swing")

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("neck_count", 3)
        self.declare_parameter("neck_amplitude", 0.3)
        self.declare_parameter("neck_frequency", 0.25)
        self.declare_parameter("neck_phase_step", 0.0)
        self.declare_parameter("eye_amplitude", 0.3)
        self.declare_parameter("eye_frequency", 0.25)
        self.declare_parameter("eye_phase_step", 0.0)
        self.declare_parameter("startup_read_timeout", 2.0)
        self.declare_parameter("ramp_duration", 3.0)
        self.declare_parameter("startup_enable", True)
        self.declare_parameter("set_startup_modes", True)
        self.declare_parameter("eye_names", ["leyelow", "reyelow", "leyeup", "reyeup"])

        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.neck_count = max(1, int(self.get_parameter("neck_count").value))
        self.neck_amplitude = min(0.3, abs(float(self.get_parameter("neck_amplitude").value)))
        self.neck_frequency = abs(float(self.get_parameter("neck_frequency").value))
        self.neck_phase_step = float(self.get_parameter("neck_phase_step").value)
        self.eye_amplitude = min(0.3, abs(float(self.get_parameter("eye_amplitude").value)))
        self.eye_frequency = abs(float(self.get_parameter("eye_frequency").value))
        self.eye_phase_step = float(self.get_parameter("eye_phase_step").value)
        self.startup_read_timeout = max(0.0, float(self.get_parameter("startup_read_timeout").value))
        self.ramp_duration = max(0.0, float(self.get_parameter("ramp_duration").value))
        self.eye_names = list(self.get_parameter("eye_names").value)

        self.capture_initial_states = True
        self.neck_state_ready = False
        self.eye_state_ready = False
        self.neck_initial_positions = [0.0] * self.neck_count
        self.eye_initial_positions = [0.0] * len(self.eye_names)

        self.neck_pub = self.create_publisher(JointState, "/can1/command", 10)
        self.eye_pub = self.create_publisher(JointState, "/can2/command", 10)
        self.neck_sub = self.create_subscription(
            JointState, "/can1/joint_states", self.on_neck_state, 10)
        self.eye_sub = self.create_subscription(
            JointState, "/can2/joint_states", self.on_eye_state, 10)

        self.wait_for_initial_positions()
        self.capture_initial_states = False
        self.configure_startup()
        self.start_time = self.get_clock().now()

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.tick)

    def on_neck_state(self, msg):
        if not self.capture_initial_states or len(msg.position) < self.neck_count:
            return

        self.neck_initial_positions = [
            float(msg.position[i]) for i in range(self.neck_count)
        ]
        self.neck_state_ready = True

    def on_eye_state(self, msg):
        if not self.capture_initial_states:
            return

        positions = None
        if msg.name:
            by_name = {
                name: float(msg.position[i])
                for i, name in enumerate(msg.name)
                if i < len(msg.position)
            }
            if all(name in by_name for name in self.eye_names):
                positions = [by_name[name] for name in self.eye_names]
        elif len(msg.position) >= len(self.eye_names):
            positions = [float(msg.position[i]) for i in range(len(self.eye_names))]

        if positions is None:
            return

        self.eye_initial_positions = positions
        self.eye_state_ready = True

    def wait_for_initial_positions(self):
        deadline = time.monotonic() + self.startup_read_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.neck_state_ready and self.eye_state_ready:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.neck_state_ready:
            self.get_logger().info(f"neck startup positions: {self.neck_initial_positions}")
        else:
            self.get_logger().warn("no /can1/joint_states received; neck starts from zero command")

        if self.eye_state_ready:
            self.get_logger().info(f"eye startup positions: {self.eye_initial_positions}")
        else:
            self.get_logger().warn("no /can2/joint_states received; eyes start from zero command")

    def configure_startup(self):
        if bool(self.get_parameter("set_startup_modes").value):
            self.call_set_mode("/can1/set_mode", [], 0)
            self.call_set_mode("/can2/set_mode", self.eye_names, 4)

        if bool(self.get_parameter("startup_enable").value):
            self.call_trigger("/can1/enable")
            self.call_trigger("/can2/enable")

    def call_set_mode(self, service_name, names, mode):
        client = self.create_client(SetMotorMode, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{service_name} not available; continuing with topic commands")
            return

        req = SetMotorMode.Request()
        req.names = names
        req.mode = int(mode)
        future = client.call_async(req)
        future.add_done_callback(lambda fut, name=service_name: self.log_service_result(name, fut))

    def call_trigger(self, service_name):
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{service_name} not available; continuing with topic commands")
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda fut, name=service_name: self.log_service_result(name, fut))

    def log_service_result(self, service_name, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{service_name} failed: {exc}")
            return

        if hasattr(response, "success") and not response.success:
            self.get_logger().warn(f"{service_name}: {response.message}")

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        neck_omega = 2.0 * math.pi * self.neck_frequency
        omega = 2.0 * math.pi * self.eye_frequency
        ramp = self.ramp_alpha(elapsed)

        neck_targets = [
            self.neck_amplitude * math.sin(neck_omega * elapsed + i * self.neck_phase_step)
            for i in range(self.neck_count)
        ]
        eye_targets = [
            self.eye_amplitude * math.sin(omega * elapsed + i * self.eye_phase_step)
            for i, _ in enumerate(self.eye_names)
        ]

        neck_msg = JointState()
        neck_msg.header.stamp = now.to_msg()
        neck_msg.position = self.blend(self.neck_initial_positions, neck_targets, ramp)
        neck_msg.velocity = [0.0] * self.neck_count
        neck_msg.effort = [0.0] * self.neck_count
        self.neck_pub.publish(neck_msg)

        eye_msg = JointState()
        eye_msg.header.stamp = now.to_msg()
        eye_msg.name = self.eye_names
        eye_msg.position = self.blend(self.eye_initial_positions, eye_targets, ramp)
        eye_msg.velocity = [0.0] * len(self.eye_names)
        eye_msg.effort = [0.0] * len(self.eye_names)
        self.eye_pub.publish(eye_msg)

    def ramp_alpha(self, elapsed):
        if self.ramp_duration <= 0.0:
            return 1.0
        x = min(1.0, max(0.0, elapsed / self.ramp_duration))
        return x * x * (3.0 - 2.0 * x)

    @staticmethod
    def blend(start, target, alpha):
        return [
            (1.0 - alpha) * start[i] + alpha * target[i]
            for i in range(min(len(start), len(target)))
        ]


def main():
    rclpy.init()
    node = NeckZeroEyeSwing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
