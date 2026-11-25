#!/usr/bin/env python3
#
# Combined Joint Controller + Cartesian Arm Teleop + Swerve Base Driver (Tkinter + ROS2)
#

# ====================================================
# EASY-TO-TUNE GLOBAL SETTINGS
# ====================================================
# (step size removed — use direct entry for precise values)
# ====================================================

# =========================
# ARM TRAJECTORY SETTINGS
# =========================
ARM_VEL = 0.5   # rad/s — lower = slower arm motions
ARM_ACC = 1.0   # rad/s² — lower = gentler acceleration
# =========================

# =========================
# SAFETY GATES (no-jump guarantees)
# =========================
# Every trajectory must start from a FRESH MEASURED joint position, never from a
# guess. If we have no recent /joint_states for a joint, we refuse to send rather
# than command it from a stale/assumed value (which would make the follower JTC
# step across the gap on the first tick -> servo overload -> red error LED).
STATE_TIMEOUT_SEC = 0.5   # max age of a measurement before a send is refused
# =========================

# =========================
# BASE DRIVER SETTINGS
# =========================
BASE_TOPIC   = "/cmd_vel"
BASE_LIN_VEL = 0.30      # m/s (x and y)
BASE_ANG_VEL = 0.80      # rad/s
PUBLISH_HZ   = 20.0
BOOST_MULT   = 2.0       # speed multiplier while Shift is held
SLOW_MULT        = 0.37   # linear speed multiplier while Ctrl is held  (~0.11 m/s)
SLOW_ANG_MULT    = 0.15   # angular speed multiplier while Ctrl is held (~0.12 rad/s)
# =========================

# Sign pattern to mirror one arm's joints onto the other (joints 1..7; gripper copied).
# Derived from the SG2 URDF: both arms share identical joint axes and their origins differ
# only by a flipped Y. Reflecting across the sagittal (x-z) plane, a rotation about axis a
# maps to one about M*a with negated angle (M = diag(1,-1,1)). Matching that per joint:
#   axis Y -> reflects to -Y -> COPY ;  axis X or Z -> preserved -> NEGATE
#   j1(Y)+  j2(X)-  j3(Z)-  j4(Y)+  j5(Z)-  j6(Y)+  j7(X)-
ARM_MIRROR_SIGNS = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
# =========================

import os
import re
import sys
import json
import math
import argparse
import tkinter as tk
from tkinter import ttk
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, BatteryState, CompressedImage
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

try:
    from ffw_teleop.cartesian_servo_teleop import CartesianServoTeleop
except ImportError:
    from cartesian_servo_teleop import CartesianServoTeleop


class CombinedKeyboardController(Node):
    def __init__(self, cams=False, cam_h=420, ref_head=None, ref_right=None, ref_left=None):
        super().__init__("combined_keyboard_controller")

        # -----------------------------
        # Tk root
        # -----------------------------
        self.root = tk.Tk()
        self.root.title("Robot Controller (Joints + Cartesian + Base)")

        # Battery status bar — always visible at the bottom of the window.
        # Must be packed BEFORE the notebook so it claims its space first; the
        # notebook's expand=True then fills whatever remains above it.
        bat_bar = tk.Frame(self.root, relief="sunken", bd=1)
        bat_bar.pack(side="bottom", fill="x")
        self._bat_labels = {}
        for side in ("left", "right"):
            lbl = tk.Label(bat_bar, text=f"BAT {side.upper()}: --",
                           font=("Courier", 10), fg="#888888", padx=12, pady=3)
            lbl.pack(side="left")
            self._bat_labels[side] = lbl

        # Persistent STOP bar — always visible regardless of active tab.
        stop_bar = tk.Frame(self.root, relief="raised", bd=2, bg="#cc2222")
        stop_bar.pack(side="bottom", fill="x")
        tk.Button(
            stop_bar,
            text="■  STOP ALL  (S)",
            font=("Arial", 11, "bold"),
            bg="#cc2222", fg="white",
            activebackground="#ff4444", activeforeground="white",
            relief="flat", padx=16, pady=4,
            command=self.stop_all,
        ).pack(side="left", padx=4, pady=2)

        # Optional camera row (--cams): left wrist | head | right wrist, above the tabs
        if cams:
            self._build_cam_panel(cam_h, ref_head, ref_right, ref_left)

        # Tab container
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True)

        self.joints_tab = ttk.Frame(self.notebook)
        self.cartesian_tab = ttk.Frame(self.notebook)
        self.base_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.joints_tab, text="Joints")
        self.notebook.add(self.cartesian_tab, text="Cartesian")
        self.notebook.add(self.base_tab, text="Base")

        # -----------------------------
        # Joint controller state
        # -----------------------------
        self.presets = {}  # name -> {ctrl_key: [positions]}
        self.preset_name_var = tk.StringVar()
        self.selected_preset_var = tk.StringVar(value="No presets")

        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
        except NameError:
            base_dir = os.getcwd()

        self.preset_dir = os.path.join(base_dir, "preset_joint_positions")
        os.makedirs(self.preset_dir, exist_ok=True)

        # Trajectory sampling
        self.time_step = 0.01  # seconds between trajectory points

        # Controllers config
        self.controllers = {
            "arm_l": {
                "joints": [
                    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3",
                    "arm_l_joint4", "arm_l_joint5", "arm_l_joint6",
                    "arm_l_joint7", "gripper_l_joint1"
                ],
                "positions": [0.0] * 8,
                "entries": [None] * 8,
                "actual_labels": [None] * 8,
                "scales": [None] * 8,
                "limits": [
                    (-3.14, 3.14),    # arm_l_joint1
                    (0.0, 3.14),      # arm_l_joint2
                    (-3.14, 3.14),    # arm_l_joint3
                    (-2.9361, 1.0786),# arm_l_joint4
                    (-3.14, 3.14),    # arm_l_joint5
                    (-1.57, 1.57),    # arm_l_joint6
                    (-1.8201, 1.5804),# arm_l_joint7
                    (0.0, 1.1),       # gripper_l_joint1
                ],
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
                    10
                ),
                "last_positions": [0.0] * 8,
                # Measured state, written ONLY by joint_state_callback. None = never measured.
                "actual_positions": [None] * 8,
                "actual_stamp": [None] * 8,
                "vel": ARM_VEL,
                "acc": ARM_ACC,
            },
            "arm_r": {
                "joints": [
                    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3",
                    "arm_r_joint4", "arm_r_joint5", "arm_r_joint6",
                    "arm_r_joint7", "gripper_r_joint1"
                ],
                "positions": [0.0] * 8,
                "entries": [None] * 8,
                "actual_labels": [None] * 8,
                "scales": [None] * 8,
                "limits": [
                    (-3.14, 3.14),    # arm_r_joint1
                    (-3.14, 0.0),     # arm_r_joint2
                    (-3.14, 3.14),    # arm_r_joint3
                    (-2.9361, 1.0786),# arm_r_joint4
                    (-3.14, 3.14),    # arm_r_joint5
                    (-1.57, 1.57),    # arm_r_joint6
                    (-1.5804, 1.8201),# arm_r_joint7
                    (0.0, 1.1),       # gripper_r_joint1
                ],
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
                    10
                ),
                "last_positions": [0.0] * 8,
                # Measured state, written ONLY by joint_state_callback. None = never measured.
                "actual_positions": [None] * 8,
                "actual_stamp": [None] * 8,
                "vel": ARM_VEL,
                "acc": ARM_ACC,
            },
            "head": {
                "joints": ["head_joint1", "head_joint2"],
                "positions": [0.0] * 2,
                "entries": [None] * 2,
                "actual_labels": [None] * 2,
                "scales": [None] * 2,
                "limits": [
                    (-0.2317, 0.75),    # head_joint1 (bumped from URDF 0.6951; hw reaches it)
                    (-0.35, 0.35),      # head_joint2
                ],
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joystick_controller_left/joint_trajectory",
                    10
                ),
                "last_positions": [0.0] * 2,
                # Measured state, written ONLY by joint_state_callback. None = never measured.
                "actual_positions": [None] * 2,
                "actual_stamp": [None] * 2,
                "vel": 0.5,
                "acc": 1.0,
            },
            "lift": {
                "joints": ["lift_joint"],
                "positions": [0.0],
                "entries": [None],
                "actual_labels": [None],
                "scales": [None],
                "limits": [(-0.5, 0.0)],  # lift_joint (prismatic)
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joystick_controller_right/joint_trajectory",
                    10
                ),
                "last_positions": [0.0],
                # Measured state, written ONLY by joint_state_callback. None = never measured.
                "actual_positions": [None],
                "actual_stamp": [None],
                "vel": 0.05,
                "acc": 0.2,
            },
        }

        # Pre-build joint name -> (ctrl_key, index) map for O(1) callback lookup
        self._joint_map = {}
        for ctrl_key, ctrl in self.controllers.items():
            for i, joint in enumerate(ctrl["joints"]):
                self._joint_map[joint] = (ctrl_key, i)

        # JointState subscription
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )
        self.joint_received = False

        # Battery subscriptions (FFW-SG2 only; FFW-BG2 has no batteries)
        for side in ("left", "right"):
            self.create_subscription(
                BatteryState, f"/ai_worker/battery/{side}/state",
                lambda msg, s=side: self._on_battery(s, msg), 10)

        # -----------------------------
        # Base driver state
        # -----------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.base_pub = self.create_publisher(Twist, BASE_TOPIC, qos)

        self.cartesian_position_step_var = tk.StringVar(value="0.010")
        self.cartesian_rotation_step_var = tk.StringVar(value="5.0")
        self.cartesian_status_var = tk.StringVar(value="Cartesian teleop is initializing...")
        self.cartesian_pose_vars = {
            "arm_l": {
                "current": tk.StringVar(value="Current:\nnot synced"),
                "target": tk.StringVar(value="Target:\nnot synced"),
            },
            "arm_r": {
                "current": tk.StringVar(value="Current:\nnot synced"),
                "target": tk.StringVar(value="Target:\nnot synced"),
            },
        }
        self.cartesian = CartesianServoTeleop(
            node=self,
            arm_configs={
                "arm_l": {
                    "group_name": "arm_l",
                    "ee_link": "end_effector_l_link",
                },
                "arm_r": {
                    "group_name": "arm_r",
                    "ee_link": "end_effector_r_link",
                },
            },
            joint_names_callback=self.get_cartesian_joint_names,
            seed_positions_callback=self.get_cartesian_seed_positions,
            command_callback=self.send_cartesian_arm_command,
            reference_frame="arm_base_link",
        )
        self.cartesian_status_var.set(self.cartesian.get_status_message())

        self._keys = {
            "Up": False,
            "Down": False,
            "Left": False,
            "Right": False,
            "A": False,
            "D": False,
            "Shift": False,
            "Ctrl": False,
        }

        self._running = True

        # Timer for publishing cmd_vel
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cmd)

        # -----------------------------
        # Build UI
        # -----------------------------
        self.build_joints_gui(self.joints_tab)
        self.build_cartesian_gui(self.cartesian_tab)
        self.build_base_gui(self.base_tab)
        self.refresh_cartesian_gui()

        # Load presets after GUI exists
        self.load_presets_from_disk()
        self.update_preset_menu()

        # Initialize last_positions
        for ctrl in self.controllers.values():
            ctrl["last_positions"] = list(ctrl["positions"])

        # -----------------------------
        # Global key bindings
        # -----------------------------
        # Important: these bind to the root so they work across tabs (while window has focus).
        self._bind_base_keys()
        self._bind_joint_keys()

        # Window close button
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)


    # =========================================================
    # JOINTS: callbacks / behavior
    # =========================================================
    def joint_state_callback(self, msg: JointState):
        # O(1) lookup via pre-built maps
        msg_index = {name: idx for idx, name in enumerate(msg.name)}

        for joint_name, (ctrl_key, i) in self._joint_map.items():
            idx = msg_index.get(joint_name)
            if idx is None:
                continue
            value = msg.position[idx]
            ctrl = self.controllers[ctrl_key]

            ctrl["last_positions"][i] = value
            # Measured state used to anchor every trajectory's start point. This is the
            # ONLY place actual_positions/actual_stamp are written, so they always reflect
            # the real robot (never an assumed target).
            ctrl["actual_positions"][i] = value
            ctrl["actual_stamp"][i] = self.get_clock().now()

            if ctrl["actual_labels"][i]:
                ctrl["actual_labels"][i].config(text=f"{value:.3f}")

            if not self.joint_received:
                ctrl["positions"][i] = value
                if ctrl["entries"][i]:
                    ctrl["entries"][i].delete(0, tk.END)
                    ctrl["entries"][i].insert(0, f"{value:.3f}")
                if ctrl["scales"][i] is not None:
                    ctrl["scales"][i].set(value)

        self.joint_received = True

    def _on_battery(self, side, msg):
        pct = int(msg.percentage * 100)
        v = msg.voltage
        color = "#22aa44" if pct > 30 else "#cc8800" if pct > 15 else "#cc2222"
        lbl = self._bat_labels.get(side)
        if lbl:
            lbl.config(text=f"BAT {side.upper()}: {pct}%  {v:.1f} V", fg=color)

    def _fresh_actual(self, ctrl_key):
        """Return (ok, measured_positions, reason) for a controller.

        ok is True only if every joint has a measurement (actual_positions != None)
        no older than STATE_TIMEOUT_SEC. This is the gate that guarantees a trajectory
        starts from where the robot really is — if we can't prove that, we refuse to
        send rather than command from a guess and make the JTC step across the gap.
        """
        ctrl = self.controllers[ctrl_key]
        now = self.get_clock().now()
        joints = ctrl["joints"]

        missing = [joints[i] for i, p in enumerate(ctrl["actual_positions"]) if p is None]
        if missing:
            return False, None, f"never measured: {', '.join(missing)}"

        stale = []
        for i, stamp in enumerate(ctrl["actual_stamp"]):
            if stamp is None:
                stale.append(joints[i])
                continue
            age = (now - stamp).nanoseconds * 1e-9
            if age > STATE_TIMEOUT_SEC:
                stale.append(f"{joints[i]} ({age:.2f}s)")
        if stale:
            return False, None, f"stale /joint_states: {', '.join(stale)}"

        return True, list(ctrl["actual_positions"]), ""

    def create_smooth_trajectory(self, joint_names, start_pos, end_pos, vel, acc):
        # NO-JUMP INVARIANT: callers pass start_pos = the freshly MEASURED joint
        # positions (see send_command). The first point is then at t=0 with position ==
        # measured and zero velocity (quintic), so the follower JTC starts exactly where
        # the robot is and has no gap to step across. header.stamp is left at zero
        # ("start now", clock-skew-safe). If start_pos were a guess, the JTC would jump
        # the difference on the first tick — the bug this whole design prevents.
        traj = JointTrajectory()
        traj.joint_names = joint_names

        if not joint_names:
            return traj

        start = np.array(start_pos)
        end = np.array(end_pos)
        deltas = end - start
        max_delta = np.max(np.abs(deltas))

        if max_delta < 1e-6:
            point = JointTrajectoryPoint()
            point.positions = end.tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(0.1 * 1e9)
            traj.points.append(point)
            return traj

        vel = max(vel, 1e-3)
        acc = max(acc, 1e-3)

        # Duration is set by the single largest joint delta so that the fastest
        # joint moves at the same peak speed (~vel) in EVERY motion — concentrated
        # and multi-joint moves alike. (Using the norm here would let concentrated
        # single-joint moves run faster than multi-joint ones.)
        T_from_vel = max_delta * 1.875 / vel
        T_from_acc = (max_delta * 5.7735026 / acc) ** 0.5
        duration = max(T_from_vel, T_from_acc, 0.1)

        num_points = max(int(duration / self.time_step) + 1, 2)
        times = np.linspace(0.0, duration, num_points)

        # Vectorized quintic polynomial: (num_points,) arrays
        t_norm = times / duration
        t2 = t_norm ** 2
        t3 = t2 * t_norm
        t4 = t3 * t_norm
        t5 = t4 * t_norm

        pos_coeffs = 10 * t3 - 15 * t4 + 6 * t5
        vel_coeffs = (30 * t2 - 60 * t3 + 30 * t4) / duration
        acc_coeffs = (60 * t_norm - 180 * t2 + 120 * t3) / (duration * duration)

        # Broadcast to (num_points, num_joints)
        all_positions = start + np.outer(pos_coeffs, deltas)
        all_velocities = np.outer(vel_coeffs, deltas)
        all_accelerations = np.outer(acc_coeffs, deltas)

        for k, t in enumerate(times):
            point = JointTrajectoryPoint()
            point.positions = all_positions[k].tolist()
            point.velocities = all_velocities[k].tolist()
            point.accelerations = all_accelerations[k].tolist()
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t % 1.0) * 1e9)
            traj.points.append(point)

        self.get_logger().info(
            f"Trajectory: duration={duration:.3f}s, max_delta={max_delta:.3f} rad, "
            f"points={len(traj.points)}, vel={vel:.3f}, acc={acc:.3f}"
        )
        return traj

    def send_command(self, ctrl_key, exclude_gripper=False):
        """Send a smooth trajectory to the target, anchored to the freshly MEASURED
        start so the follower JTC has no gap to step across. Returns True if a motion
        was published, False if the send was refused for safety.

        exclude_gripper=True drops the gripper joint from the published trajectory entirely
        (used by cartesian jogs). A JTC honours a partial-joint goal and leaves unlisted
        joints under their existing command, so a cartesian move cannot touch the gripper
        at all — not even to re-assert a setpoint."""
        ctrl = self.controllers[ctrl_key]

        # Gate 1: refuse unless we have a fresh measurement for every joint.
        ok, start, reason = self._fresh_actual(ctrl_key)
        if not ok:
            msg = f"[SAFE-SKIP] {ctrl_key}: {reason}"
            self.get_logger().warn(msg)
            self.cartesian_status_var.set(msg)
            return False

        target = list(ctrl["positions"])
        joints = ctrl["joints"]

        # Cartesian: drop the gripper (last joint) so it is never part of the goal.
        if exclude_gripper:
            joints = joints[:-1]
            start = start[:-1]
            target = target[:-1]

        # No per-step clamp: targets are already clamped to joint limits everywhere they're
        # set (sliders/entries/presets/IK/mirror), so a move can never exceed a joint's range.
        # A large-but-legal move is safe here because the trajectory starts from the MEASURED
        # pose and is velocity/accel capped — it just takes longer, it does not jump.

        msg = self.create_smooth_trajectory(
            joints,
            start,            # measured actual — never a guess
            target,
            ctrl["vel"],
            ctrl["acc"],
        )
        ctrl["publisher"].publish(msg)
        # NOTE: deliberately do NOT write last_positions/actual_positions here. The next
        # trajectory must start from MEASURED state (joint_state_callback owns it), not an
        # assumed target — assuming completion is exactly what caused the fast-slew faults.
        self.get_logger().info(f"[SEND] {ctrl_key} -> {target}")
        return True

    def mirror_arm(self, src_key, dst_key, send=True):
        """Copy the source arm's commanded joint pose onto the destination arm as a
        sagittal-plane mirror image (see ARM_MIRROR_SIGNS), clamp to the destination's
        limits, refresh its GUI, and send it."""
        src = self.controllers[src_key]
        dst = self.controllers[dst_key]
        for i in range(len(dst["joints"])):
            sign = ARM_MIRROR_SIGNS[i] if i < len(ARM_MIRROR_SIGNS) else 1.0  # gripper: copy
            val = sign * src["positions"][i]
            min_l, max_l = dst["limits"][i]
            clamped = max(min(val, max_l), min_l)
            dst["positions"][i] = clamped
            if dst["entries"][i]:
                dst["entries"][i].delete(0, tk.END)
                dst["entries"][i].insert(0, f"{clamped:.3f}")
            if dst["scales"][i] is not None:
                dst["scales"][i].set(clamped)
        self.get_logger().info(f"[MIRROR] {src_key} -> {dst_key} (signs={ARM_MIRROR_SIGNS})")
        if send:
            self.send_command(dst_key)

    def sync_targets_to_current(self):
        """Set every command target (entries + sliders) to the latest measured joint
        positions, so the targets match the robot's current state and nothing jumps on
        the next send."""
        for ctrl in self.controllers.values():
            for i in range(len(ctrl["joints"])):
                min_l, max_l = ctrl["limits"][i]
                clamped = max(min(ctrl["last_positions"][i], max_l), min_l)
                ctrl["positions"][i] = clamped
                if ctrl["entries"][i]:
                    ctrl["entries"][i].delete(0, tk.END)
                    ctrl["entries"][i].insert(0, f"{clamped:.3f}")
                if ctrl["scales"][i] is not None:
                    ctrl["scales"][i].set(clamped)
        self.get_logger().info("[SYNC] targets set to current actual positions")

    def send_all(self, event=None):
        self.get_logger().info("[SEND ALL] triggered")
        keys = list(self.controllers.keys())
        sent = [k for k in keys if self.send_command(k)]
        skipped = [k for k in keys if k not in sent]
        if skipped:
            self.get_logger().warn(
                f"[SEND ALL] sent {len(sent)}/{len(keys)}; skipped (unsafe): {', '.join(skipped)}"
            )

    def on_joint_slider_change(self, ctrl_key, joint_index, value):
        ctrl = self.controllers[ctrl_key]
        try:
            v = float(value)
        except ValueError:
            return
        ctrl["positions"][joint_index] = v
        if ctrl["entries"][joint_index]:
            entry = ctrl["entries"][joint_index]
            entry.delete(0, tk.END)
            entry.insert(0, f"{v:.3f}")

    def on_joint_entry_submit(self, ctrl_key, joint_index):
        ctrl = self.controllers[ctrl_key]
        entry = ctrl["entries"][joint_index]
        text = entry.get().strip()
        try:
            v = float(text)
        except ValueError:
            self.get_logger().warn(f"[ENTRY] Invalid value: '{text}'")
            return
        min_l, max_l = ctrl["limits"][joint_index]
        v = max(min(v, max_l), min_l)
        ctrl["positions"][joint_index] = v
        entry.delete(0, tk.END)
        entry.insert(0, f"{v:.3f}")
        if ctrl["scales"][joint_index] is not None:
            ctrl["scales"][joint_index].set(v)

    # =========================================================
    # CARTESIAN: callbacks / behavior
    # =========================================================
    def get_cartesian_joint_names(self, arm_key):
        return list(self.controllers[arm_key]["joints"][:7])

    def get_cartesian_seed_positions(self, arm_key):
        return list(self.controllers[arm_key]["last_positions"][:7])

    def send_cartesian_arm_command(self, arm_key, arm_joint_positions):
        ctrl = self.controllers[arm_key]

        for i, value in enumerate(arm_joint_positions):
            min_l, max_l = ctrl["limits"][i]
            clamped = max(min(float(value), max_l), min_l)
            ctrl["positions"][i] = clamped

            if ctrl["scales"][i] is not None:
                ctrl["scales"][i].set(clamped)
            if ctrl["entries"][i]:
                ctrl["entries"][i].delete(0, tk.END)
                ctrl["entries"][i].insert(0, f"{clamped:.3f}")

        # Cartesian IK only solves the 7 arm joints — it says nothing about the gripper.
        # Publish a 7-joint trajectory (exclude_gripper) so the gripper is never in the goal
        # and a cartesian jog cannot open, close, or even re-command it. The gripper keeps
        # whatever it was last told (via its slider/entry/preset).
        self.send_command(arm_key, exclude_gripper=True)

    def set_cartesian_status(self, message, log_level="warn"):
        self.cartesian_status_var.set(message)
        if log_level == "error":
            self.get_logger().error(message)
        elif log_level == "info":
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def refresh_cartesian_gui(self):
        for arm_key in ("arm_l", "arm_r"):
            current_text, target_text = self.cartesian.get_pose_texts(arm_key)
            self.cartesian_pose_vars[arm_key]["current"].set(current_text)
            self.cartesian_pose_vars[arm_key]["target"].set(target_text)

        self.cartesian_status_var.set(self.cartesian.get_status_message())

    def _parse_cartesian_step(self, text_var, label, degrees=False):
        text = text_var.get().strip()

        try:
            value = float(text)
        except ValueError:
            self.set_cartesian_status(f"{label} must be numeric, got '{text}'.")
            return None

        if value <= 0.0:
            self.set_cartesian_status(f"{label} must be > 0, got {value}.")
            return None

        if degrees:
            return math.radians(value)
        return value

    def cartesian_sync_arm(self, arm_key):
        self.cartesian.sync_to_current_pose(arm_key)
        self.refresh_cartesian_gui()

    def cartesian_sync_all(self):
        for arm_key in ("arm_l", "arm_r"):
            self.cartesian.sync_to_current_pose(arm_key)
        self.refresh_cartesian_gui()

    def cartesian_jog(self, arm_key, axis, direction):
        kwargs = {
            "dx": 0.0,
            "dy": 0.0,
            "dz": 0.0,
            "droll": 0.0,
            "dpitch": 0.0,
            "dyaw": 0.0,
        }

        if axis in ("x", "y", "z"):
            step = self._parse_cartesian_step(
                self.cartesian_position_step_var,
                "Cartesian position step",
            )
            if step is None:
                return
            kwargs[f"d{axis}"] = direction * step
        else:
            step = self._parse_cartesian_step(
                self.cartesian_rotation_step_var,
                "Cartesian rotation step",
                degrees=True,
            )
            if step is None:
                return
            kwargs[f"d{axis}"] = direction * step

        self.cartesian.jog(arm_key, **kwargs)
        self.refresh_cartesian_gui()

    # -----------------------------
    # Presets: disk + memory
    # -----------------------------
    def _preset_name_to_filename(self, name: str) -> str:
        parts = name.split("/", 1)
        if len(parts) == 2:
            folder, fname = parts
            safe_folder = re.sub(r"[^A-Za-z0-9_\-]+", "_", folder).strip("_") or "preset"
            safe_fname  = re.sub(r"[^A-Za-z0-9_\-]+", "_", fname).strip("_")  or "preset"
            subdir = os.path.join(self.preset_dir, safe_folder)
            os.makedirs(subdir, exist_ok=True)
            return os.path.join(subdir, f"{safe_fname}.json")
        safe = re.sub(r"[^A-Za-z0-9_\-]+", "_", name).strip("_") or "preset"
        return os.path.join(self.preset_dir, f"{safe}.json")

    def load_presets_from_disk(self):
        if not os.path.isdir(self.preset_dir):
            return

        def _load(path, key):
            try:
                with open(path, "r") as f:
                    data = json.load(f)
                snapshot = data["positions"] if isinstance(data, dict) and "positions" in data else data
                if isinstance(snapshot, dict):
                    self.presets[key] = snapshot
                    self.get_logger().info(f"[PRESET LOAD DISK] Loaded '{key}' from {path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load preset from {path}: {e}")

        # Root-level files
        for fname in sorted(os.listdir(self.preset_dir)):
            if not fname.endswith(".json"):
                continue
            fpath = os.path.join(self.preset_dir, fname)
            if os.path.isfile(fpath):
                _load(fpath, os.path.splitext(fname)[0])

        # One level of subdirectories
        for entry in sorted(os.listdir(self.preset_dir)):
            subdir = os.path.join(self.preset_dir, entry)
            if not os.path.isdir(subdir):
                continue
            for fname in sorted(os.listdir(subdir)):
                if not fname.endswith(".json"):
                    continue
                fpath = os.path.join(subdir, fname)
                if os.path.isfile(fpath):
                    _load(fpath, f"{entry}/{os.path.splitext(fname)[0]}")

        if self.presets and self.selected_preset_var.get() == "No presets":
            self.selected_preset_var.set(sorted(self.presets.keys())[0])

    def save_preset(self):
        name = self.preset_name_var.get().strip()
        if not name:
            self.get_logger().warn("[PRESET] Empty name, not saving.")
            return

        snapshot = {k: list(c["positions"]) for k, c in self.controllers.items()}
        self.presets[name] = snapshot

        filename = self._preset_name_to_filename(name)
        data = {"positions": snapshot}
        try:
            text = json.dumps(data, indent=2)
            text = re.sub(
                r'\[\n((?:\s+-?[\d.]+,?\n)+)\s*\]',
                lambda m: '[' + ', '.join(x.strip().rstrip(',') for x in m.group(1).splitlines() if x.strip()) + ']',
                text,
            )
            with open(filename, "w") as f:
                f.write(text)
            self.get_logger().info(f"[PRESET SAVED] '{name}' -> {filename}")
        except Exception as e:
            self.get_logger().error(f"[PRESET SAVE ERROR] {e}")

        self.update_preset_menu()
        self.selected_preset_var.set(name)

    def load_preset(self):
        name = self.selected_preset_var.get()
        if name not in self.presets:
            self.get_logger().warn(f"[PRESET LOAD] '{name}' not found.")
            return

        snapshot = self.presets[name]
        for ctrl_key, positions in snapshot.items():
            if ctrl_key in self.controllers:
                ctrl = self.controllers[ctrl_key]
                for i, val in enumerate(positions):
                    if i < len(ctrl["positions"]):
                        min_l, max_l = ctrl["limits"][i]
                        clamped = max(min(val, max_l), min_l)
                        ctrl["positions"][i] = clamped
                        if ctrl["entries"][i]:
                            ctrl["entries"][i].delete(0, tk.END)
                            ctrl["entries"][i].insert(0, f"{clamped:.3f}")
                        if ctrl["scales"][i] is not None:
                            ctrl["scales"][i].set(clamped)

        self.get_logger().info(f"[PRESET LOADED] {name}")

    def update_preset(self):
        name = self.selected_preset_var.get()
        if name not in self.presets:
            self.get_logger().warn("[PRESET UPDATE] No preset selected.")
            return
        snapshot = {k: list(c["positions"]) for k, c in self.controllers.items()}
        self.presets[name] = snapshot
        filename = self._preset_name_to_filename(name)
        data = {"positions": snapshot}
        try:
            text = json.dumps(data, indent=2)
            text = re.sub(
                r'\[\n((?:\s+-?[\d.]+,?\n)+)\s*\]',
                lambda m: '[' + ', '.join(x.strip().rstrip(',') for x in m.group(1).splitlines() if x.strip()) + ']',
                text,
            )
            with open(filename, "w") as f:
                f.write(text)
            self.get_logger().info(f"[PRESET UPDATED] '{name}' -> {filename}")
        except Exception as e:
            self.get_logger().error(f"[PRESET UPDATE ERROR] {e}")

    def update_preset_menu(self):
        menu = self.preset_menu["menu"]
        menu.delete(0, "end")
        # Hold Python refs to the submenus so they aren't garbage-collected while shown.
        self._preset_submenus = []

        if not self.presets:
            self.selected_preset_var.set("No presets")
            menu.add_command(
                label="No presets",
                command=lambda v="No presets": self.selected_preset_var.set(v),
            )
            return

        current = self.selected_preset_var.get()
        if current not in self.presets:
            current = sorted(self.presets.keys())[0]
        self.selected_preset_var.set(current)

        # Group keys by folder ("" = ungrouped). Each folder becomes a cascade submenu so
        # the top-level list stays short (one row per folder) instead of one row per preset.
        groups = {}
        for name in sorted(self.presets.keys()):
            folder = name.split("/", 1)[0] if "/" in name else ""
            groups.setdefault(folder, []).append(name)

        # Ungrouped presets sit directly on the top-level menu.
        for name in groups.pop("", []):
            menu.add_command(label=name, command=lambda v=name: self.selected_preset_var.set(v))

        if groups and menu.index("end") is not None:
            menu.add_separator()

        # One cascade per folder; its presets (short names) live inside the submenu.
        for folder in sorted(groups):
            submenu = tk.Menu(menu, tearoff=0)
            self._preset_submenus.append(submenu)
            for name in groups[folder]:
                label = name.split("/", 1)[1]
                submenu.add_command(label=label,
                                    command=lambda v=name: self.selected_preset_var.set(v))
            menu.add_cascade(label=f"{folder}  ({len(groups[folder])})", menu=submenu)

    # =========================================================
    # BASE: callbacks / behavior
    # =========================================================
    def _is_text_widget(self, widget):
        # While focus is in a text field, base keys must not drive the robot.
        return isinstance(widget, (tk.Entry, ttk.Entry, tk.Spinbox, tk.Text))

    def _on_base_key(self, key: str, pressed: bool, event):
        # Ignore presses while typing in an entry; ALWAYS honor releases so a key
        # can never get stuck True (e.g. if its release lands in a text field).
        if pressed and self._is_text_widget(getattr(event, "widget", None)):
            return
        self._set_key(key, pressed)

    def _set_key(self, key: str, pressed: bool):
        if key in self._keys:
            self._keys[key] = pressed

    def _publish_cmd(self):
        # If we're shutting down, keep publishing a zero command once stop() runs.
        if not self._running:
            return

        lin_x = 0.0
        lin_y = 0.0
        ang_z = 0.0

        if self._keys.get("Ctrl", False):
            mult = SLOW_MULT
        elif self._keys.get("Shift", False):
            mult = BOOST_MULT
        else:
            mult = 1.0
        lin_vel = BASE_LIN_VEL * mult
        ang_vel = BASE_ANG_VEL * (SLOW_ANG_MULT if self._keys.get("Ctrl", False) else mult)

        if self._keys["Up"]:
            lin_x += lin_vel
        if self._keys["Down"]:
            lin_x -= lin_vel
        if self._keys["A"]:
            lin_y += lin_vel   # left strafe
        if self._keys["D"]:
            lin_y -= lin_vel   # right strafe

        steering_dir = -1.0 if lin_x < 0 else 1.0

        if self._keys["Left"]:
            ang_z += ang_vel * steering_dir
        if self._keys["Right"]:
            ang_z -= ang_vel * steering_dir

        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.linear.y = float(lin_y)
        msg.angular.z = float(ang_z)
        self.base_pub.publish(msg)

        if hasattr(self, "base_status_var"):
            if lin_x == 0.0 and lin_y == 0.0 and ang_z == 0.0:
                self.base_status_var.set("Stopped")
            else:
                boost_txt = " (SLOW)" if mult == SLOW_MULT else (" (BOOST)" if mult == BOOST_MULT else "")
                rev_txt = " (REV STEER)" if steering_dir < 0 else ""
                self.base_status_var.set(
                    f"x={lin_x:.2f} m/s, y={lin_y:.2f} m/s, yaw={ang_z:.2f} rad/s{boost_txt}{rev_txt}"
                )

    def stop_base(self):
        for k in self._keys:
            self._keys[k] = False
        self.base_pub.publish(Twist())
        if hasattr(self, "base_status_var"):
            self.base_status_var.set("Stopped")
        self.get_logger().info("[BASE] STOP")

    def stop_arms(self):
        """Cancel any running arm trajectory. If we have a fresh measurement, hold at the
        MEASURED position (single point in 0.1 s) which overrides whatever was executing.
        If we don't, publish an EMPTY trajectory — the JTC treats that as cancel/hold-in-
        place — rather than commanding a guessed hold that could itself cause a step."""
        for ctrl_key, ctrl in self.controllers.items():
            traj = JointTrajectory()
            traj.joint_names = ctrl["joints"]

            ok, measured, _ = self._fresh_actual(ctrl_key)
            if ok:
                point = JointTrajectoryPoint()
                point.positions = list(measured)
                point.time_from_start.sec = 0
                point.time_from_start.nanosec = int(0.1 * 1e9)
                traj.points.append(point)
            # else: leave traj.points empty -> JTC cancels the active goal and holds.

            ctrl["publisher"].publish(traj)
        self.get_logger().info("[ARMS] STOP")

    def stop_all(self):
        self.stop_base()
        self.stop_arms()

    # =========================================================
    # GUI builders
    # =========================================================
    def build_joints_gui(self, parent):
        row = 0

        tk.Label(parent, text="GLOBAL CONTROLS", font=("Arial", 12, "bold"))\
            .grid(row=row, column=0, columnspan=4, pady=(8, 0))
        row += 1

        tk.Button(parent, text="SEND ALL (Spacebar)", command=self.send_all)\
            .grid(row=row, column=0, columnspan=4, pady=(2, 8), sticky="we")
        row += 1

        tk.Button(parent, text="Set targets = current state",
                  command=self.sync_targets_to_current)\
            .grid(row=row, column=0, columnspan=4, pady=(0, 8), sticky="we")
        row += 1

        # Mirror one arm's pose onto the other as a sagittal-plane symmetric pose, then send.
        mirror_frame = tk.Frame(parent)
        mirror_frame.grid(row=row, column=0, columnspan=4, sticky="we", pady=(0, 8))
        tk.Button(mirror_frame, text="Move RIGHT to mirror LEFT",
                  command=lambda: self.mirror_arm("arm_l", "arm_r"))\
            .pack(side="left", expand=True, fill="x", padx=(0, 3))
        tk.Button(mirror_frame, text="Move LEFT to mirror RIGHT",
                  command=lambda: self.mirror_arm("arm_r", "arm_l"))\
            .pack(side="left", expand=True, fill="x", padx=(3, 0))
        row += 1

        tk.Label(parent, text="Preset name").grid(row=row, column=0, sticky="w")
        tk.Entry(parent, textvariable=self.preset_name_var)\
            .grid(row=row, column=1, sticky="we")
        tk.Button(parent, text="Save", command=self.save_preset)\
            .grid(row=row, column=2, sticky="we")
        tk.Button(parent, text="Load", command=self.load_preset)\
            .grid(row=row, column=3, sticky="we")
        row += 1

        tk.Label(parent, text="Presets").grid(row=row, column=0, sticky="w")
        self.preset_menu = tk.OptionMenu(parent, self.selected_preset_var, "No presets")
        self.preset_menu.grid(row=row, column=1, columnspan=2, sticky="we")
        tk.Button(parent, text="Update", command=self.update_preset)\
            .grid(row=row, column=3, sticky="we", pady=(0, 10))
        row += 1

        controllers_frame = tk.Frame(parent)
        controllers_frame.grid(row=row, column=0, columnspan=4, sticky="nsew")
        row += 1

        positions = {
            "arm_l": (0, 0),
            "arm_r": (0, 1),
            "head":  (1, 0),
            "lift":  (1, 1),
        }

        for ctrl_key, (r, c) in positions.items():
            ctrl = self.controllers[ctrl_key]

            frame = tk.Frame(controllers_frame, borderwidth=1, relief="groove", padx=5, pady=5)
            frame.grid(row=r, column=c, padx=5, pady=5, sticky="n")

            tk.Label(frame, text=ctrl_key.upper(), font=("Arial", 11, "bold"))\
                .grid(row=0, column=0, columnspan=5, pady=(0, 5))

            # Column headers
            tk.Label(frame, text="Joint", font=("Arial", 9, "bold"))\
                .grid(row=1, column=0, sticky="w")
            tk.Label(frame, text="Command", font=("Arial", 9, "bold"))\
                .grid(row=1, column=1, columnspan=2)
            tk.Label(frame, text="Cmd", font=("Arial", 9, "bold"))\
                .grid(row=1, column=3)
            tk.Label(frame, text="Actual", font=("Arial", 9, "bold"), fg="blue")\
                .grid(row=1, column=4)

            row_local = 2
            for i, joint in enumerate(ctrl["joints"]):
                tk.Label(frame, text=joint).grid(row=row_local, column=0, sticky="w")

                min_l, max_l = ctrl["limits"][i]
                scale = tk.Scale(
                    frame,
                    from_=min_l,
                    to=max_l,
                    orient="horizontal",
                    length=180,
                    resolution=0.001,
                    showvalue=False,
                    command=lambda v, c=ctrl_key, j=i: self.on_joint_slider_change(c, j, v),
                )
                scale.set(ctrl["positions"][i])
                scale.grid(row=row_local, column=1, columnspan=2, sticky="we")

                entry = tk.Entry(frame, width=8)
                entry.insert(0, f"{ctrl['positions'][i]:.3f}")
                entry.bind("<Return>", lambda e, c=ctrl_key, j=i: self.on_joint_entry_submit(c, j))
                entry.grid(row=row_local, column=3, sticky="e")

                actual_label = tk.Label(frame, text="---", width=8, fg="blue")
                actual_label.grid(row=row_local, column=4, sticky="e")

                ctrl["entries"][i] = entry
                ctrl["scales"][i] = scale
                ctrl["actual_labels"][i] = actual_label

                row_local += 1

            tk.Button(
                frame,
                text=f"SEND {ctrl_key.upper()}",
                command=lambda c=ctrl_key: self.send_command(c),
            ).grid(row=row_local, column=0, columnspan=5, pady=(5, 0), sticky="we")

    def build_cartesian_gui(self, parent):
        parent.grid_columnconfigure(0, weight=1)

        info = (
            "Servo-style Cartesian arm teleop for arm_l and arm_r.\n"
            "Requires MoveIt move_group to provide /compute_fk and /compute_ik.\n"
            f"Reference frame: {self.cartesian.reference_frame}\n"
            "Translation (robot frame):  +X front ↑ / -X back ↓    +Y left ← / -Y right →    +Z up ↑ / -Z down ↓\n"
            "Rotation:  Roll = lean L/R (← / →)    Pitch = tilt fwd/back (↓ / ↑)    Yaw = wrist faces L/R (← / →)"
        )
        tk.Label(parent, text=info, padx=12, pady=10, justify="left")\
            .grid(row=0, column=0, sticky="w")

        controls = tk.Frame(parent)
        controls.grid(row=1, column=0, sticky="we", padx=12)

        tk.Label(controls, text="Position step [m]").grid(row=0, column=0, sticky="w")
        tk.Entry(controls, textvariable=self.cartesian_position_step_var, width=8)\
            .grid(row=0, column=1, padx=(4, 12), sticky="w")

        tk.Label(controls, text="Rotation step [deg]").grid(row=0, column=2, sticky="w")
        tk.Entry(controls, textvariable=self.cartesian_rotation_step_var, width=8)\
            .grid(row=0, column=3, padx=(4, 12), sticky="w")

        tk.Button(controls, text="Sync All", command=self.cartesian_sync_all)\
            .grid(row=0, column=4, padx=(4, 0), sticky="w")

        tk.Label(
            parent,
            textvariable=self.cartesian_status_var,
            padx=12,
            pady=6,
            justify="left",
            wraplength=720,
        ).grid(row=2, column=0, sticky="w")

        panels = tk.Frame(parent)
        panels.grid(row=3, column=0, padx=12, pady=(0, 10), sticky="nsew")

        self.build_cartesian_arm_panel(panels, "arm_l", 0, 0)
        self.build_cartesian_arm_panel(panels, "arm_r", 0, 1)

    def build_cartesian_arm_panel(self, parent, arm_key, row, column):
        frame = tk.Frame(parent, borderwidth=1, relief="groove", padx=8, pady=8)
        frame.grid(row=row, column=column, padx=5, pady=5, sticky="n")

        button_state = "normal" if self.cartesian.available else "disabled"

        tk.Label(frame, text=arm_key.upper(), font=("Arial", 11, "bold"))\
            .grid(row=0, column=0, columnspan=3, pady=(0, 6))

        tk.Label(
            frame,
            textvariable=self.cartesian_pose_vars[arm_key]["current"],
            justify="left",
            anchor="w",
            width=34,
        ).grid(row=1, column=0, columnspan=3, sticky="w")

        tk.Label(
            frame,
            textvariable=self.cartesian_pose_vars[arm_key]["target"],
            justify="left",
            anchor="w",
            width=34,
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(4, 6))

        tk.Button(
            frame,
            text="Sync To Current",
            state=button_state,
            command=lambda a=arm_key: self.cartesian_sync_arm(a),
        ).grid(row=3, column=0, columnspan=3, sticky="we", pady=(0, 6))

        # Each row: the LEFT button is the down/back/left direction, the RIGHT button the
        # up/front/right direction — so the button's side matches the motion it commands.
        # X: +X front (↑) / -X back (↓)
        self.add_cartesian_jog_row(
            frame, 4, "X (front/back)", "↓ back  -X", "+X  front ↑",
            lambda a=arm_key: self.cartesian_jog(a, "x", -1.0),
            lambda a=arm_key: self.cartesian_jog(a, "x", 1.0),
            button_state,
        )
        # Y: +Y left (←) / -Y right (→)
        self.add_cartesian_jog_row(
            frame, 5, "Y (left/right)", "← left  +Y", "-Y  right →",
            lambda a=arm_key: self.cartesian_jog(a, "y", 1.0),
            lambda a=arm_key: self.cartesian_jog(a, "y", -1.0),
            button_state,
        )
        # Z: +Z up (↑) / -Z down (↓)
        self.add_cartesian_jog_row(
            frame, 6, "Z (up/down)", "↓ down  -Z", "+Z  up ↑",
            lambda a=arm_key: self.cartesian_jog(a, "z", -1.0),
            lambda a=arm_key: self.cartesian_jog(a, "z", 1.0),
            button_state,
        )
        self.add_cartesian_jog_row(
            frame, 7, "Roll (lean L/R)", "← lean", "lean →",
            lambda a=arm_key: self.cartesian_jog(a, "roll", -1.0),
            lambda a=arm_key: self.cartesian_jog(a, "roll", 1.0),
            button_state,
        )
        self.add_cartesian_jog_row(
            frame, 8, "Pitch (fwd/back)", "↓ back", "fwd ↑",
            lambda a=arm_key: self.cartesian_jog(a, "pitch", -1.0),
            lambda a=arm_key: self.cartesian_jog(a, "pitch", 1.0),
            button_state,
        )
        # Yaw: +Yaw turns wrist to face left (←) / -Yaw right (→)
        self.add_cartesian_jog_row(
            frame, 9, "Yaw (wrist L/R)", "← +Yaw", "-Yaw →",
            lambda a=arm_key: self.cartesian_jog(a, "yaw", 1.0),
            lambda a=arm_key: self.cartesian_jog(a, "yaw", -1.0),
            button_state,
        )

    def add_cartesian_jog_row(self, parent, row, label, left_text, right_text, left_cmd, right_cmd, state):
        # left_* is placed in the left button column, right_* in the right — so a button
        # that means "left"/"down"/"back" sits on the left and "right"/"up"/"front" on the right.
        tk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        tk.Button(parent, text=left_text, width=12, state=state, command=left_cmd)\
            .grid(row=row, column=1, padx=2, pady=2)
        tk.Button(parent, text=right_text, width=12, state=state, command=right_cmd)\
            .grid(row=row, column=2, padx=2, pady=2)

    def build_base_gui(self, parent):
        info = (
            f"Publishing to: {BASE_TOPIC}\n\n"
            "Controls:\n"
            "  ↑ / ↓   forward / backward\n"
            "  A / D   strafe left / right\n"
            "  ← / →   rotate left / right\n"
            "  Shift   BOOST (hold)\n"
            "  Ctrl    SLOW ~0.11 m/s (hold)\n\n"
            "S = STOP ALL     Esc = STOP ALL + Quit     Space = Send All joints\n\n"
            "Tip: click this window (or anywhere in the app) to ensure it has keyboard focus."
        )
        tk.Label(parent, text=info, padx=12, pady=10, justify="left").pack(anchor="w")

        self.base_status_var = tk.StringVar(value="Stopped")
        tk.Label(parent, textvariable=self.base_status_var, padx=12, pady=6).pack(anchor="w")

        row = tk.Frame(parent)
        row.pack(pady=(0, 10), anchor="w")

        tk.Button(row, text="STOP ALL", width=10, command=self.stop_all).pack(side="left", padx=6)
        tk.Button(row, text="Quit", width=10, command=self.shutdown).pack(side="left", padx=6)

    # =========================================================
    # Key bindings
    # =========================================================
    def _bind_base_keys(self):
        # Press (suppressed while a text field has focus — see _on_base_key)
        self.root.bind("<KeyPress-Up>",    lambda e: self._on_base_key("Up", True, e))
        self.root.bind("<KeyPress-Down>",  lambda e: self._on_base_key("Down", True, e))
        self.root.bind("<KeyPress-Left>",  lambda e: self._on_base_key("Left", True, e))
        self.root.bind("<KeyPress-Right>", lambda e: self._on_base_key("Right", True, e))

        self.root.bind("<KeyPress-a>", lambda e: self._on_base_key("A", True, e))
        self.root.bind("<KeyPress-d>", lambda e: self._on_base_key("D", True, e))

        self.root.bind("<KeyPress-Shift_L>", lambda e: self._on_base_key("Shift", True, e))
        self.root.bind("<KeyPress-Shift_R>", lambda e: self._on_base_key("Shift", True, e))

        self.root.bind("<KeyPress-Control_L>", lambda e: self._on_base_key("Ctrl", True, e))
        self.root.bind("<KeyPress-Control_R>", lambda e: self._on_base_key("Ctrl", True, e))

        # Release (always honored so a key can never stay stuck True)
        self.root.bind("<KeyRelease-Up>",    lambda e: self._set_key("Up", False))
        self.root.bind("<KeyRelease-Down>",  lambda e: self._set_key("Down", False))
        self.root.bind("<KeyRelease-Left>",  lambda e: self._set_key("Left", False))
        self.root.bind("<KeyRelease-Right>", lambda e: self._set_key("Right", False))

        self.root.bind("<KeyRelease-a>", lambda e: self._set_key("A", False))
        self.root.bind("<KeyRelease-d>", lambda e: self._set_key("D", False))

        self.root.bind("<KeyRelease-Shift_L>", lambda e: self._set_key("Shift", False))
        self.root.bind("<KeyRelease-Shift_R>", lambda e: self._set_key("Shift", False))

        self.root.bind("<KeyRelease-Control_L>", lambda e: self._set_key("Ctrl", False))
        self.root.bind("<KeyRelease-Control_R>", lambda e: self._set_key("Ctrl", False))

        # S = stop base (without quitting); ESC = stop + quit
        self.root.bind("<KeyPress-s>",  lambda e: None if self._is_text_widget(e.widget) else self.stop_all())
        self.root.bind("<Escape>", lambda e: self.shutdown())

    def _bind_joint_keys(self):
        # If you still want Space to mean "SEND ALL" for joints:
        # That conflicts with base stop. Instead, map SEND ALL to Ctrl+Space by default.
        # (You can swap this if you prefer.)
        self.root.bind("<KeyRelease-space>", self.send_all)

    # =========================================================
    # Camera panel (--cams): live views to drive the base / arms by
    # =========================================================
    CAM_VIEWS = [  # (title, topic, rotate the raw D405 90 deg CCW so the jaws are at the bottom)
        ("left wrist topic",  "/camera_left/camera_left/color/image_rect_raw/compressed",   True),
        ("head (ZED left)",   "/zed/zed_node/left/image_rect_color/compressed",             False),
        ("right wrist topic", "/camera_right/camera_right/color/image_rect_raw/compressed", True),
    ]

    def _build_cam_panel(self, cam_h, ref_head, ref_right=None, ref_left=None):
        import cv2
        from PIL import Image, ImageTk
        self._cv2, self._Image, self._ImageTk = cv2, Image, ImageTk
        self._cam_h = int(cam_h)
        self._cam_raw = {}      # title -> latest compressed bytes (decoded only at the panel's refresh rate)
        self._cam_stamp = {}    # title -> wall time of the latest frame
        self._cam_imgs = {}     # title -> the PhotoImage kept alive for its label
        self._cam_labels = {}
        self._ref_edges = None
        if ref_head:
            ref = cv2.imread(ref_head)
            if ref is None:
                self.get_logger().warn(f"--ref_head: cannot read {ref_head}")
            else:
                self._ref_edges = cv2.Canny(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY), 40, 120)
                self.get_logger().info(f"head view: overlaying the edges of {ref_head} (green)")
        # one row as wide as the controller below it: three equal cells (grid, uniform columns); each view is
        # fitted into its cell with its aspect kept (black bars), so all three cells have the same size
        # wrist references (already upright, e.g. a generated start frame): blended 50/50 over the live upright view
        self._ref_wrist = {}
        for key, path in (("right", ref_right), ("left", ref_left)):
            if path:
                ref = cv2.imread(path)
                if ref is None:
                    self.get_logger().warn(f"--ref_{key}: cannot read {path}")
                else:
                    self._ref_wrist[key] = ref
                    self.get_logger().info(f"{key} wrist view: 50/50 with {path}")
        panel = tk.Frame(self.root, relief="sunken", bd=1, bg="black")
        panel.pack(side="top", fill="x")
        self._cam_panel = panel
        for i, (title, topic, rot) in enumerate(self.CAM_VIEWS):
            panel.grid_columnconfigure(i, weight=1, uniform="cams")
            tk.Label(panel, text=title, font=("Arial", 9), bg="black", fg="white").grid(row=0, column=i, sticky="ew")
            lbl = tk.Label(panel, text=f"waiting for\n{topic}", bg="black", fg="gray", font=("Arial", 8))
            lbl.grid(row=1, column=i, sticky="nsew", padx=1, pady=1)
            self._cam_labels[title] = lbl
            self.create_subscription(CompressedImage, topic,
                                     lambda m, t=title: self._on_cam(t, m), qos_profile_sensor_data)
        self.root.after(100, self._refresh_cams)

    def _on_cam(self, title, msg):
        self._cam_raw[title] = bytes(msg.data)
        self._cam_stamp[title] = self.get_clock().now()

    def _refresh_cams(self):
        if not self._running:
            return
        cv2, now = self._cv2, self.get_clock().now()
        for title, topic, rot in self.CAM_VIEWS:
            data = self._cam_raw.pop(title, None)
            if data is None:
                continue
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if rot:
                img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            side = "right" if title.startswith("right") else "left" if title.startswith("left") else None
            if side in self._ref_wrist:
                ref = cv2.resize(self._ref_wrist[side], (img.shape[1], img.shape[0]), interpolation=cv2.INTER_AREA)
                img = cv2.addWeighted(img, 0.5, ref, 0.5, 0)
            if title.startswith("head") and self._ref_edges is not None:
                e = cv2.resize(self._ref_edges, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
                img[e > 0] = (0, 255, 0)
            # cell = a third of the controller's width (the notebook below sets it; the row never widens the window)
            cw = max(120, (max(self.notebook.winfo_width(), 360) - 12) // len(self.CAM_VIEWS)); ch = self._cam_h
            f = min(cw / img.shape[1], ch / img.shape[0]); w, h = max(1, int(img.shape[1] * f)), max(1, int(img.shape[0] * f))
            cell = np.zeros((ch, cw, 3), np.uint8); y0, x0 = (ch - h) // 2, (cw - w) // 2
            cell[y0:y0 + h, x0:x0 + w] = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            photo = self._ImageTk.PhotoImage(self._Image.fromarray(cv2.cvtColor(cell, cv2.COLOR_BGR2RGB)))
            self._cam_imgs[title] = photo
            self._cam_labels[title].configure(image=photo, text="", width=cw, height=ch)
        self.root.after(100, self._refresh_cams)   # ~10 Hz

    # =========================================================
    # Main loop / shutdown
    # =========================================================
    def shutdown(self):
        if not self._running:
            return
        self._running = False
        self.stop_all()
        self.get_logger().info("[SHUTDOWN] Closing GUI/controller...")
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self):
        # Wait for first /joint_states to sync GUI, but don’t block forever if absent.
        # We’ll wait a little while, then continue anyway.
        tries = 0
        while not self.joint_received and rclpy.ok() and self._running and tries < 10:
            self.get_logger().info("Waiting for /joint_states (initial sync)...")
            rclpy.spin_once(self, timeout_sec=0.5)
            tries += 1

        self.get_logger().info(
            "GUI Ready. Joints tab: sliders + SEND. Cartesian tab: MoveIt FK/IK jogging. "
            "Base tab: keyboard driving. "
            "Keyboard focus requires clicking the window. "
            "SAFETY: arm sends are gated on fresh /joint_states and anchored to the "
            "measured pose — a send is refused (no motion) if state is stale/missing."
        )

        try:
            while rclpy.ok() and self._running:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.root.update()
        except tk.TclError:
            self._running = False
        finally:
            self.stop_base()


def main():
    ap = argparse.ArgumentParser(description="Joints / Cartesian / Base controller")
    ap.add_argument("--cams", action="store_true", help="show the left wrist / head / right wrist cameras above the tabs")
    ap.add_argument("--cam_h", type=int, default=420, help="camera row height (px); each view fills a third of the window width")
    ap.add_argument("--ref_head", default=None, help="an image whose edges are drawn (green) over the head view, "
                    "e.g. the expected view at a planned stance: drive until the edges line up")
    ap.add_argument("--ref_right", default=None, help="an upright image blended 50/50 over the right wrist view "
                    "(e.g. the generated start frame: move the objects / base until the two copies merge)")
    ap.add_argument("--ref_left", default=None, help="the same for the left wrist view")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = CombinedKeyboardController(cams=a.cams, cam_h=a.cam_h, ref_head=a.ref_head, ref_right=a.ref_right, ref_left=a.ref_left)
    try:
        node.run()
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
