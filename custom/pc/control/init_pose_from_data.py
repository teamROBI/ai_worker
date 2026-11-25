#!/usr/bin/env python3
"""
Read a LeRobot v2 dataset, move robot to an initial pose, then compare
the live head camera view against the dataset's reference frame.

Usage:
    python3 init_pose_from_data.py <dataset_name> [--mode random|average] [--episode N] [--no-visual]

    dataset_name is resolved under <repo>/lerobot/ACS_ROBI/<dataset_name>.
    Override with --data-root or LEROBOT_DATA_ROOT on another device.

Examples:
    python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot
    python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot --mode average
    python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot --episode 5
    python3 init_pose_from_data.py ffw_sg2_rev1_pour_pot --no-visual
"""

import os
import sys
import json
import time
import re
import random
import argparse
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq
from skimage.metrics import structural_similarity as ssim

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, CompressedImage


# =========================================================
# Config
# =========================================================
HEAD_CAM_TOPIC = "/zed/zed_node/left/image_rect_color/compressed"

CONTROLLER_SLICES = {
    "arm_l": slice(0, 8),
    "arm_r": slice(8, 16),
    "head":  slice(16, 18),
    "lift":  slice(18, 19),
}



# =========================================================
# LeRobot v2 dataset reader
# =========================================================
def load_dataset_info(dataset_path):
    meta_path = os.path.join(dataset_path, "meta", "info.json")
    if not os.path.exists(meta_path):
        print(f"ERROR: Not a valid LeRobot v2 dataset: {meta_path} not found")
        sys.exit(1)
    with open(meta_path) as f:
        return json.load(f)


def read_episodes_meta(dataset_path):
    """Read episode metadata (tasks, lengths) from episodes.jsonl."""
    episodes_path = os.path.join(dataset_path, "meta", "episodes.jsonl")
    episodes = []
    if os.path.exists(episodes_path):
        with open(episodes_path) as f:
            for line in f:
                episodes.append(json.loads(line))
    return episodes


def read_init_poses(dataset_path, info):
    """Read first-frame observation.state from every episode."""
    total_episodes = info["total_episodes"]
    data_path_template = info["data_path"]
    chunks_size = info.get("chunks_size", 1000)

    state_names = info["features"]["observation.state"].get("names")
    print(f"Dataset: {total_episodes} episodes, state dim: {info['features']['observation.state']['shape']}")
    if state_names:
        print(f"State joints: {state_names}")

    init_poses = []
    for ep_idx in range(total_episodes):
        ep_chunk = ep_idx // chunks_size
        parquet_rel = data_path_template.format(episode_chunk=ep_chunk, episode_index=ep_idx)
        parquet_path = os.path.join(dataset_path, parquet_rel)

        if not os.path.exists(parquet_path):
            print(f"  WARNING: {parquet_path} not found, skipping episode {ep_idx}")
            continue

        table = pq.read_table(parquet_path, columns=["frame_index", "observation.state"])
        states = table.column("observation.state")
        init_poses.append(np.array(states[0].as_py(), dtype=np.float32))

    print(f"Loaded init poses from {len(init_poses)} episodes")
    return init_poses


def select_init_pose(init_poses, mode="random", episode_idx=None):
    """Select an init pose. Returns (state_vector, reference_episode_idx)."""
    if episode_idx is not None:
        if episode_idx < 0 or episode_idx >= len(init_poses):
            print(f"ERROR: Episode {episode_idx} out of range [0, {len(init_poses) - 1}]")
            sys.exit(1)
        print(f"Using init pose from episode {episode_idx}")
        return init_poses[episode_idx], episode_idx

    if mode == "average":
        pose = np.mean(init_poses, axis=0)
        # Find closest episode for reference image
        dists = [np.linalg.norm(p - pose) for p in init_poses]
        closest = int(np.argmin(dists))
        print(f"Using average init pose (closest episode for reference image: {closest})")
        return pose, closest

    idx = random.randint(0, len(init_poses) - 1)
    print(f"Randomly selected episode {idx}")
    return init_poses[idx], idx


def extract_reference_frame(dataset_path, info, episode_idx):
    """Extract the first frame from the episode's head camera video."""
    video_path_template = info["video_path"]
    chunks_size = info.get("chunks_size", 1000)
    ep_chunk = episode_idx // chunks_size

    video_rel = video_path_template.format(
        episode_chunk=ep_chunk,
        episode_index=episode_idx,
        video_key="observation.images.cam_head",
    )
    video_path = os.path.join(dataset_path, video_rel)

    if not os.path.exists(video_path):
        print(f"WARNING: Video not found: {video_path}")
        return None

    cap = cv2.VideoCapture(video_path)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        print(f"WARNING: Could not read first frame from {video_path}")
        return None

    print(f"Reference frame from: {video_rel} ({frame.shape[1]}x{frame.shape[0]})")
    return frame


def state_to_controller_positions(state_vector):
    """Map a 22-element state vector to controller position dicts."""
    positions = {}
    for ctrl_key, s in CONTROLLER_SLICES.items():
        positions[ctrl_key] = state_vector[s].tolist()
    return positions


# =========================================================
# Image comparison
# =========================================================
def compare_images(reference, live):
    """Compare two images. Returns (ssim_score, grid_2x2)."""
    # Resize live to match reference if needed
    if reference.shape[:2] != live.shape[:2]:
        live = cv2.resize(live, (reference.shape[1], reference.shape[0]))

    # Convert to grayscale for SSIM
    ref_gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    live_gray = cv2.cvtColor(live, cv2.COLOR_BGR2GRAY)

    score, diff = ssim(ref_gray, live_gray, full=True)
    # diff is float64 in [0, 1], where 1 = identical
    diff_uint8 = (255 - (diff * 255)).astype(np.uint8)

    # Color heatmap: red = big difference, blue = small
    heatmap = cv2.applyColorMap(diff_uint8, cv2.COLORMAP_JET)

    # Overlay heatmap on live image for regions with significant diff
    threshold = 50
    mask = diff_uint8 > threshold
    diff_overlay = live.copy()
    diff_overlay[mask] = cv2.addWeighted(live, 0.4, heatmap, 0.6, 0)[mask]

    # Blend: reference and live hovering together
    blend = cv2.addWeighted(reference, 0.5, live, 0.5, 0)

    # --- 2x2 grid ---
    h, w = reference.shape[:2]
    pad = 4
    label_h = 32
    cell_h = label_h + h
    grid = np.zeros((cell_h * 2 + pad, w * 2 + pad, 3), dtype=np.uint8)

    font = cv2.FONT_HERSHEY_SIMPLEX
    ssim_color = (0, 255, 0) if score > 0.8 else (0, 165, 255) if score > 0.6 else (0, 0, 255)

    # Top-left: Reference
    cv2.putText(grid, "Reference", (4, label_h - 6), font, 0.6, (255, 255, 255), 2)
    grid[label_h:label_h + h, 0:w] = reference

    # Top-right: Live
    x1 = w + pad
    cv2.putText(grid, "Live", (x1 + 4, label_h - 6), font, 0.6, (255, 255, 255), 2)
    grid[label_h:label_h + h, x1:x1 + w] = live

    # Bottom-left: Diff Overlay
    y1 = cell_h + pad
    cv2.putText(grid, f"Diff  SSIM={score:.3f}", (4, y1 + label_h - 6), font, 0.6, ssim_color, 2)
    grid[y1 + label_h:y1 + label_h + h, 0:w] = diff_overlay

    # Bottom-right: Ref + Live Blend
    cv2.putText(grid, "Ref + Live Blend", (x1 + 4, y1 + label_h - 6), font, 0.6, (255, 255, 255), 2)
    grid[y1 + label_h:y1 + label_h + h, x1:x1 + w] = blend

    return score, grid


# =========================================================
# ROS2 Node
# =========================================================
class InitPosePlayer(Node):
    def __init__(self):
        super().__init__("init_pose_player")

        self.controllers = {
            "arm_l": {
                "joints": [
                    "arm_l_joint1", "arm_l_joint2", "arm_l_joint3",
                    "arm_l_joint4", "arm_l_joint5", "arm_l_joint6",
                    "arm_l_joint7", "gripper_l_joint1"
                ],
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
                "vel": 1.0,
                "acc": 2.0,
            },
            "arm_r": {
                "joints": [
                    "arm_r_joint1", "arm_r_joint2", "arm_r_joint3",
                    "arm_r_joint4", "arm_r_joint5", "arm_r_joint6",
                    "arm_r_joint7", "gripper_r_joint1"
                ],
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
                "vel": 1.0,
                "acc": 2.0,
            },
            "head": {
                "joints": ["head_joint1", "head_joint2"],
                "limits": [
                    (-0.2317, 0.6951),  # head_joint1
                    (-0.35, 0.35),      # head_joint2
                ],
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
                "limits": [(-0.5, 0.0)],  # lift_joint (prismatic)
                "publisher": self.create_publisher(
                    JointTrajectory,
                    "/leader/joystick_controller_right/joint_trajectory",
                    10
                ),
                "last_positions": [0.0],
                "vel": 0.1,
                "acc": 0.4,
            },
        }

        self.time_step = 0.01
        self.joint_received = False
        self.live_frame = None

        self._joint_map = {}
        for ctrl_key, ctrl in self.controllers.items():
            for i, joint in enumerate(ctrl["joints"]):
                self._joint_map[joint] = (ctrl_key, i)

        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, HEAD_CAM_TOPIC, self.image_callback, image_qos
        )

    def joint_state_callback(self, msg):
        msg_index = {name: idx for idx, name in enumerate(msg.name)}
        for joint_name, (ctrl_key, i) in self._joint_map.items():
            idx = msg_index.get(joint_name)
            if idx is None:
                continue
            value = msg.position[idx]
            ctrl = self.controllers[ctrl_key]
            min_l, max_l = ctrl["limits"][i]
            ctrl["last_positions"][i] = max(min(value, max_l), min_l)
        self.joint_received = True

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.live_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"Image decode failed: {e}")

    def create_smooth_trajectory(self, joint_names, start_pos, end_pos, vel, acc):
        traj = JointTrajectory()
        traj.joint_names = joint_names

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

        T_from_vel = max_delta * 1.875 / vel
        T_from_acc = (max_delta * 5.7735026 / acc) ** 0.5
        duration = max(T_from_vel, T_from_acc, 0.1)

        num_points = max(int(duration / self.time_step) + 1, 2)
        times = np.linspace(0.0, duration, num_points)

        t_norm = times / duration
        t2 = t_norm ** 2
        t3 = t2 * t_norm
        t4 = t3 * t_norm
        t5 = t4 * t_norm

        pos_coeffs = 10 * t3 - 15 * t4 + 6 * t5
        vel_coeffs = (30 * t2 - 60 * t3 + 30 * t4) / duration
        acc_coeffs = (60 * t_norm - 180 * t2 + 120 * t3) / (duration * duration)

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

    def send_command(self, ctrl_key, target_positions):
        ctrl = self.controllers[ctrl_key]
        limits = ctrl["limits"]
        clamped = [max(min(v, limits[i][1]), limits[i][0]) for i, v in enumerate(target_positions)]

        traj = self.create_smooth_trajectory(
            ctrl["joints"], ctrl["last_positions"], clamped, ctrl["vel"], ctrl["acc"]
        )
        ctrl["publisher"].publish(traj)
        ctrl["last_positions"] = list(clamped)
        self.get_logger().info(f"[SEND] {ctrl_key} -> {[round(v, 4) for v in clamped]}")

    def apply_positions(self, positions_dict):
        for ctrl_key in self.controllers:
            if ctrl_key in positions_dict:
                self.send_command(ctrl_key, positions_dict[ctrl_key])

    def check_reached(self, positions_dict, tolerance=0.05):
        """Check if all controllers reached target positions within tolerance (rad).
        Returns (reached, max_error, errors_by_controller)."""
        errors = {}
        max_err = 0.0
        for ctrl_key, target in positions_dict.items():
            ctrl = self.controllers[ctrl_key]
            current = ctrl["last_positions"]
            errs = [abs(c - t) for c, t in zip(current, target)]
            ctrl_max = max(errs)
            errors[ctrl_key] = ctrl_max
            max_err = max(max_err, ctrl_max)
        reached = max_err < tolerance
        return reached, max_err, errors


# =========================================================
# Main
# =========================================================
def main():
    parser = argparse.ArgumentParser(
        description="Move robot to init pose from LeRobot v2 dataset and compare head camera view"
    )
    parser.add_argument("dataset_name", help="Dataset name under --data-root, or an absolute dataset path")
    parser.add_argument(
        "--data-root", default=os.environ.get(
            "LEROBOT_DATA_ROOT", str(Path(__file__).resolve().parents[3] / "lerobot" / "ACS_ROBI")
        ),
        help="Directory containing datasets (default: LEROBOT_DATA_ROOT or <repo>/lerobot/ACS_ROBI)",
    )
    parser.add_argument(
        "--mode", choices=["random", "average"], default="random",
        help="Init pose selection: random episode or average of all (default: random)"
    )
    parser.add_argument(
        "--episode", type=int, default=None,
        help="Use a specific episode index (overrides --mode)"
    )
    parser.add_argument(
        "--no-visual", action="store_true",
        help="Skip visual comparison (joints only)"
    )
    args = parser.parse_args()

    data_root = os.path.abspath(os.path.expanduser(args.data_root))
    dataset_path = os.path.normpath(os.path.join(data_root, args.dataset_name))

    # Auto-resolve short task names (e.g. "pour_pot" -> "ffw_sg2_rev1_pour_pot")
    if not os.path.isdir(dataset_path) and os.path.isdir(os.path.normpath(data_root)):
        candidates = [
            d for d in sorted(os.listdir(os.path.normpath(data_root)))
            if os.path.isdir(os.path.join(data_root, d)) and d.endswith(args.dataset_name)
        ]
        if len(candidates) == 1:
            print(f"Resolved '{args.dataset_name}' -> '{candidates[0]}'")
            dataset_path = os.path.normpath(os.path.join(data_root, candidates[0]))
        elif len(candidates) > 1:
            print(f"ERROR: Ambiguous name '{args.dataset_name}', matches:")
            for c in candidates:
                print(f"    {c}")
            sys.exit(1)

    if not os.path.isdir(dataset_path):
        print(f"ERROR: Dataset not found: {dataset_path}")
        print(f"  Available datasets in {os.path.normpath(data_root)}:")
        if os.path.isdir(os.path.normpath(data_root)):
            for d in sorted(os.listdir(os.path.normpath(data_root))):
                if os.path.isdir(os.path.join(data_root, d)):
                    print(f"    {d}")
        sys.exit(1)

    # Load dataset
    info = load_dataset_info(dataset_path)
    episodes_meta = read_episodes_meta(dataset_path)
    init_poses = read_init_poses(dataset_path, info)
    if not init_poses:
        print("ERROR: No episodes found")
        sys.exit(1)

    state_vector, ref_episode = select_init_pose(init_poses, mode=args.mode, episode_idx=args.episode)
    positions = state_to_controller_positions(state_vector)

    # Get task name (strip robot/version prefix like ffw_sg2_rev1_)
    task_name = "unknown"
    if ref_episode < len(episodes_meta) and "tasks" in episodes_meta[ref_episode]:
        raw_task = ", ".join(episodes_meta[ref_episode]["tasks"])
        task_name = re.sub(r'^ffw_sg\d+_rev\d+_', '', raw_task).replace('_', ' ')
    print(f"\nTask: {task_name}")
    print(f"Target positions (ref episode {ref_episode}):")
    for ctrl_key, pos in positions.items():
        print(f"  {ctrl_key}: {[round(v, 4) for v in pos]}")

    # Extract reference image from dataset video
    ref_frame = None
    if not args.no_visual:
        ref_frame = extract_reference_frame(dataset_path, info, ref_episode)

    # Send to robot
    rclpy.init()
    node = InitPosePlayer()

    print("\nWaiting for /joint_states...")
    start_time = time.time()
    while rclpy.ok() and not node.joint_received:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - start_time > 5:
            print("WARNING: /joint_states not received, using zeros as start.")
            break

    max_retries = 3
    tolerance = 0.05  # rad
    for attempt in range(1, max_retries + 1):
        print(f"\nSending init pose (attempt {attempt}/{max_retries})...")
        node.apply_positions(positions)

        # Wait for trajectory to execute
        end_time = time.time() + 4.0
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Read fresh joint states
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        reached, max_err, errors = node.check_reached(positions, tolerance)
        if reached:
            print(f"Reached init pose (max error: {max_err:.4f} rad)")
            break
        else:
            print(f"NOT reached (max error: {max_err:.4f} rad, tolerance: {tolerance})")
            for ctrl_key, err in errors.items():
                if err >= tolerance:
                    print(f"  {ctrl_key}: error {err:.4f} rad")
            if attempt == max_retries:
                print("WARNING: Failed to reach init pose after all retries. Continuing anyway.")

    # Visual comparison loop
    if ref_frame is not None:
        print(f"\nVisual comparison active (subscribing to {HEAD_CAM_TOPIC})")
        print("Press 'q' or ESC to quit, 's' to save snapshot")

        # Show reference frame immediately while waiting for live
        window_title = f"Init Pose - {task_name} (ep {ref_episode})"
        cv2.imshow(window_title, ref_frame)
        cv2.waitKey(1)

        print("Waiting for live camera frame...")
        wait_start = time.time()
        while rclpy.ok() and node.live_frame is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            cv2.waitKey(1)
            elapsed = time.time() - wait_start
            if int(elapsed) % 5 == 0 and int(elapsed) > 0 and elapsed - int(elapsed) < 0.15:
                print(f"  Still waiting... ({int(elapsed)}s)")
            if elapsed > 30:
                print("WARNING: No live camera frame received after 30s. Skipping visual comparison.")
                break

        if node.live_frame is not None:
            print("Live camera connected. Showing comparison windows...")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.03)

                if node.live_frame is None:
                    continue

                score, grid = compare_images(ref_frame, node.live_frame)
                cv2.imshow(window_title, grid)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q or ESC
                    break
                elif key == ord('s'):
                    snapshot_path = f"init_pose_comparison_ep{ref_episode}.png"
                    cv2.imwrite(snapshot_path, grid)
                    print(f"Saved snapshot: {snapshot_path} (SSIM={score:.3f})")

        cv2.destroyAllWindows()
    else:
        if not args.no_visual:
            print("No reference frame available, skipping visual comparison.")

    print("Done.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
