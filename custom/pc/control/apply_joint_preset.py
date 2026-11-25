#!/usr/bin/env python3

import os
import sys
import time
import json
import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


DEFAULT_PRESET_FILE = "action_ready.json"
PRESET_DIR_NAME = "preset_joint_positions"


# ---------------------------------------------------------
# Load JSON preset file
# ---------------------------------------------------------
def load_preset_json(path: str):
    with open(path, "r") as f:
        data = json.load(f)

    # Expected:
    # {
    #   "preset_name": "...",
    #   "positions": { "arm_l": [...], ... }
    # }
    if "positions" in data:
        return data["positions"]

    # Fallback: assume dict itself contains positions
    return data


# ---------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------
class PresetPlayer(Node):
    def __init__(self):
        super().__init__('joint_preset_player')

        # Controller definitions (same as GUI)
        self.controllers = {
            "arm_l": {
                "joints": [
                    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3",
                    "arm_l_joint4", "arm_l_joint5", "arm_l_joint6",
                    "arm_l_joint7", "gripper_l_joint1"
                ],
                "limits": [(-3.14, 3.14)] * 8,
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
                    10
                ),
                "last_positions": [0.0] * 8,
                "vel": 1.0,
                "acc": 2.0,
            },
            "arm_r": {
                "joints": [
                    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3",
                    "arm_r_joint4", "arm_r_joint5", "arm_r_joint6",
                    "arm_r_joint7", "gripper_r_joint1"
                ],
                "limits": [(-3.14, 3.14)] * 8,
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
                    10
                ),
                "last_positions": [0.0] * 8,
                "vel": 1.0,
                "acc": 2.0,
            },
            "head": {
                "joints": ["head_joint1", "head_joint2"],
                "limits": [(-1.0, 1.0)] * 2,
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joystick_controller_left/joint_trajectory",
                    10
                ),
                "last_positions": [0.0] * 2,
                "vel": 0.5,
                "acc": 1.0,
            },
            "lift": {
                "joints": ["lift_joint"],
                "limits": [(-1.0, 0.0)],
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joystick_controller_right/joint_trajectory",
                    10
                ),
                "last_positions": [0.0],
                "vel": 0.2,
                "acc": 0.8,
            }
        }

        self.time_step = 0.01
        self.joint_received = False

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

    # ---------------------------------------------------------
    # Read current joint positions
    # ---------------------------------------------------------
    def joint_state_callback(self, msg):
        for ctrl_key, ctrl in self.controllers.items():
            for i, joint in enumerate(ctrl["joints"]):
                if joint in msg.name:
                    idx = msg.name.index(joint)
                    raw = msg.position[idx]
                    min_l, max_l = ctrl["limits"][i]
                    ctrl["last_positions"][i] = max(min(raw, max_l), min_l)

        self.joint_received = True

    # ---------------------------------------------------------
    # Quintic trajectory generator (same as GUI)
    # ---------------------------------------------------------
    def create_smooth_trajectory(self, joint_names, start_pos, end_pos, vel, acc):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        deltas = [e - s for s, e in zip(start_pos, end_pos)]
        max_delta = max(abs(d) for d in deltas)

        if max_delta < 1e-6:
            point = JointTrajectoryPoint()
            point.positions = list(end_pos)
            point.time_from_start.sec = 0
            traj.points.append(point)
            return traj

        vel = max(vel, 1e-3)
        acc = max(acc, 1e-3)

        max_s1 = 1.875
        max_s2 = 5.7735026

        T_from_vel = max_delta * max_s1 / vel
        T_from_acc = (max_delta * max_s2 / acc) ** 0.5
        duration = max(T_from_vel, T_from_acc)

        num_points = max(int(duration / self.time_step) + 1, 2)
        times = np.linspace(0.0, duration, num_points)

        for t in times:
            t_norm = t / duration
            t2 = t_norm**2
            t3 = t2 * t_norm
            t4 = t3 * t_norm
            t5 = t4 * t_norm

            pos_coeff = 10*t3 - 15*t4 + 6*t5
            vel_coeff = (30*t2 - 60*t3 + 30*t4) / duration
            acc_coeff = (60*t_norm - 180*t2 + 120*t3) / (duration**2)

            positions = []
            velocities = []
            accelerations = []

            for j in range(len(joint_names)):
                delta = deltas[j]
                positions.append(start_pos[j] + delta * pos_coeff)
                velocities.append(delta * vel_coeff)
                accelerations.append(delta * acc_coeff)

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t % 1) * 1e9)

            traj.points.append(point)

        return traj

    # ---------------------------------------------------------
    def send_command(self, ctrl_key, target_positions):
        ctrl = self.controllers[ctrl_key]
        start_pos = ctrl["last_positions"]

        traj = self.create_smooth_trajectory(
            ctrl["joints"],
            start_pos,
            target_positions,
            ctrl["vel"],
            ctrl["acc"]
        )

        ctrl["publisher"].publish(traj)
        ctrl["last_positions"] = list(target_positions)

        self.get_logger().info(f"[SEND] {ctrl_key} → {target_positions}")

    # ---------------------------------------------------------
    def apply_positions(self, positions_dict):
        for ctrl_key, ctrl in self.controllers.items():
            if ctrl_key not in positions_dict:
                continue

            desired = positions_dict[ctrl_key]
            limits = ctrl["limits"]

            # Clamp
            target = []
            for i, val in enumerate(desired):
                min_l, max_l = limits[i]
                target.append(max(min(val, max_l), min_l))

            self.send_command(ctrl_key, target)


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
def main():

    # Directory where presets are stored
    base_dir = os.path.dirname(os.path.abspath(__file__))
    preset_dir = os.path.join(base_dir, PRESET_DIR_NAME)

    if not os.path.isdir(preset_dir):
        print(f"ERROR: Directory not found: {preset_dir}")
        sys.exit(1)

    # Determine which preset to load
    if len(sys.argv) < 2:
        preset_name = DEFAULT_PRESET_FILE
    else:
        preset_name = sys.argv[1] + ".json"

    preset_path = os.path.join(preset_dir, preset_name)

    if not os.path.exists(preset_path):
        print(f"ERROR: Preset file not found: {preset_path}")
        sys.exit(1)

    print(f"Loading preset: {preset_path}")
    positions = load_preset_json(preset_path)

    # Start ROS
    rclpy.init()
    node = PresetPlayer()

    print("Waiting for /joint_states...")
    start = time.time()
    while rclpy.ok() and not node.joint_received:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - start > 5:
            print("WARNING: /joint_states not received, using zeros.")
            break

    node.apply_positions(positions)

    # Spin briefly to ensure publication
    end_time = time.time() + 2.0
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
