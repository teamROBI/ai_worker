#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy

try:
    from builtin_interfaces.msg import Duration as DurationMsg
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
    from moveit_msgs.srv import GetPositionFK, GetPositionIK

    IMPORT_ERROR = None
except ImportError as exc:
    DurationMsg = None
    PoseStamped = None
    MoveItErrorCodes = None
    PositionIKRequest = None
    RobotState = None
    GetPositionFK = None
    GetPositionIK = None
    IMPORT_ERROR = exc


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw))
    if norm < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return normalize_quaternion(qx, qy, qz, qw)


def quaternion_multiply(q1, q2):
    # Hamilton product: rotation q1 applied after q2. Each q is (x, y, z, w).
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_about_axis(angle, axis):
    half = angle * 0.5
    s = math.sin(half)
    ax, ay, az = axis
    return ax * s, ay * s, az * s, math.cos(half)


def euler_from_quaternion(qx, qy, qz, qw):
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)

    sinr_cosp = 2.0 * ((qw * qx) + (qy * qz))
    cosr_cosp = 1.0 - 2.0 * ((qx * qx) + (qy * qy))
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * ((qw * qy) - (qz * qx))
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * ((qw * qz) + (qx * qy))
    cosy_cosp = 1.0 - 2.0 * ((qy * qy) + (qz * qz))
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wrap_to_pi(angle):
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi:
        return math.pi
    return wrapped


@dataclass
class CartesianPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @classmethod
    def from_pose(cls, pose):
        qx, qy, qz, qw = normalize_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return cls(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    def copy(self):
        return CartesianPose(
            x=self.x,
            y=self.y,
            z=self.z,
            qx=self.qx,
            qy=self.qy,
            qz=self.qz,
            qw=self.qw,
        )


class CartesianServoTeleop:
    def __init__(
        self,
        node,
        arm_configs,
        joint_names_callback,
        seed_positions_callback,
        command_callback,
        reference_frame="arm_base_link",
        service_timeout_sec=0.5,
    ):
        self.node = node
        self.arm_configs = arm_configs
        self.joint_names_callback = joint_names_callback
        self.seed_positions_callback = seed_positions_callback
        self.command_callback = command_callback
        self.reference_frame = reference_frame
        self.service_timeout_sec = service_timeout_sec

        self.available = IMPORT_ERROR is None
        self.current_poses = {}
        self.target_poses = {}

        if not self.available:
            self._status_message = (
                "Cartesian teleop unavailable: "
                f"{IMPORT_ERROR}. Install/source MoveIt Python interfaces."
            )
            self._fk_client = None
            self._ik_client = None
            return

        self._fk_client = self.node.create_client(GetPositionFK, "/compute_fk")
        self._ik_client = self.node.create_client(GetPositionIK, "/compute_ik")
        self._status_message = (
            "Cartesian teleop ready. Launch MoveIt move_group to use "
            "/compute_fk and /compute_ik."
        )

    def get_status_message(self):
        return self._status_message

    def refresh_current_pose(self, arm_key):
        if not self.available:
            return False

        pose = self._compute_current_pose(arm_key)
        if pose is None:
            return False

        self.current_poses[arm_key] = pose
        return True

    def sync_to_current_pose(self, arm_key):
        if not self.refresh_current_pose(arm_key):
            return False

        self.target_poses[arm_key] = self.current_poses[arm_key].copy()
        self._status_message = f"{arm_key}: target synced from current end-effector pose."
        return True

    def jog(self, arm_key, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0):
        if not self.available:
            return False

        if arm_key not in self.target_poses and not self.sync_to_current_pose(arm_key):
            return False

        self.refresh_current_pose(arm_key)

        target_pose = self.target_poses[arm_key].copy()
        target_pose.x += dx
        target_pose.y += dy
        target_pose.z += dz

        # Apply rotation jogs as quaternion composition about the END-EFFECTOR's
        # OWN axes (tool frame): right-multiplying the delta (q * dq) rotates in the
        # body frame, so roll/pitch/yaw stay consistent relative to the gripper no
        # matter how the wrist is oriented. (Left-multiplying would rotate about the
        # fixed reference-frame axes, making roll/pitch appear to swap as the wrist
        # yaws.) Quaternion composition also avoids the gimbal lock that decomposing
        # to Euler at pitch = +/-90 deg would introduce.
        q = (target_pose.qx, target_pose.qy, target_pose.qz, target_pose.qw)
        for angle, axis in (
            (droll, (1.0, 0.0, 0.0)),
            (dpitch, (0.0, 1.0, 0.0)),
            (dyaw, (0.0, 0.0, 1.0)),
        ):
            if angle != 0.0:
                q = quaternion_multiply(q, quaternion_about_axis(angle, axis))

        target_pose.qx, target_pose.qy, target_pose.qz, target_pose.qw = normalize_quaternion(*q)

        arm_joint_positions = self._solve_ik(arm_key, target_pose)
        if arm_joint_positions is None:
            return False

        self.target_poses[arm_key] = target_pose
        self.command_callback(arm_key, arm_joint_positions)
        self._status_message = (
            f"{arm_key}: Cartesian jog sent in {self.reference_frame} frame."
        )
        return True

    def get_pose_texts(self, arm_key):
        current_text = self._format_pose_block("Current", self.current_poses.get(arm_key))
        target_text = self._format_pose_block("Target", self.target_poses.get(arm_key))
        return current_text, target_text

    def _format_pose_block(self, label, pose):
        if pose is None:
            return f"{label}:\nnot synced"

        roll, pitch, yaw = euler_from_quaternion(pose.qx, pose.qy, pose.qz, pose.qw)
        return (
            f"{label}:\n"
            f"xyz [{pose.x:+.3f}, {pose.y:+.3f}, {pose.z:+.3f}] m\n"
            f"rpy [{math.degrees(roll):+.1f}, {math.degrees(pitch):+.1f}, "
            f"{math.degrees(yaw):+.1f}] deg"
        )

    def _compute_current_pose(self, arm_key):
        config = self.arm_configs[arm_key]

        request = GetPositionFK.Request()
        request.header.frame_id = self.reference_frame
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.fk_link_names = [config["ee_link"]]
        request.robot_state = self._build_robot_state(arm_key)

        response = self._call_service(
            client=self._fk_client,
            request=request,
            service_name="/compute_fk",
        )
        if response is None:
            return None

        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            self._status_message = (
                f"{arm_key}: MoveIt FK failed with code {response.error_code.val}."
            )
            self.node.get_logger().warn(self._status_message)
            return None

        return CartesianPose.from_pose(response.pose_stamped[0].pose)

    def _solve_ik(self, arm_key, target_pose):
        config = self.arm_configs[arm_key]
        joint_names = list(self.joint_names_callback(arm_key))

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.reference_frame
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose.position.x = target_pose.x
        pose_stamped.pose.position.y = target_pose.y
        pose_stamped.pose.position.z = target_pose.z
        pose_stamped.pose.orientation.x = target_pose.qx
        pose_stamped.pose.orientation.y = target_pose.qy
        pose_stamped.pose.orientation.z = target_pose.qz
        pose_stamped.pose.orientation.w = target_pose.qw

        ik_request = PositionIKRequest()
        ik_request.group_name = config["group_name"]
        ik_request.ik_link_name = config["ee_link"]
        ik_request.pose_stamped = pose_stamped
        ik_request.robot_state = self._build_robot_state(arm_key)
        ik_request.avoid_collisions = False

        timeout = DurationMsg()
        timeout.sec = 0
        timeout.nanosec = int(self.service_timeout_sec * 1e9)
        ik_request.timeout = timeout

        request = GetPositionIK.Request()
        request.ik_request = ik_request

        response = self._call_service(
            client=self._ik_client,
            request=request,
            service_name="/compute_ik",
        )
        if response is None:
            return None

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._status_message = (
                f"{arm_key}: MoveIt IK failed with code {response.error_code.val}."
            )
            self.node.get_logger().warn(self._status_message)
            return None

        joint_state = response.solution.joint_state
        joint_map = dict(zip(joint_state.name, joint_state.position))
        missing_joints = [joint_name for joint_name in joint_names if joint_name not in joint_map]
        if missing_joints:
            self._status_message = (
                f"{arm_key}: IK response missing joints: {missing_joints}"
            )
            self.node.get_logger().warn(self._status_message)
            return None

        return [float(joint_map[joint_name]) for joint_name in joint_names]

    def _build_robot_state(self, arm_key):
        joint_names = list(self.joint_names_callback(arm_key))
        joint_positions = list(self.seed_positions_callback(arm_key))

        if len(joint_names) != len(joint_positions):
            raise ValueError(
                f"{arm_key}: joint name/position length mismatch: "
                f"{len(joint_names)} vs {len(joint_positions)}"
            )

        robot_state = RobotState()
        robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        robot_state.joint_state.name = joint_names
        robot_state.joint_state.position = joint_positions
        return robot_state

    def _call_service(self, client, request, service_name):
        if not client.wait_for_service(timeout_sec=0.2):
            self._status_message = (
                f"{service_name} is unavailable. Launch ffw_moveit_config move_group first."
            )
            self.node.get_logger().warn(self._status_message)
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=self.service_timeout_sec,
        )

        if not future.done():
            self._status_message = f"{service_name} timed out."
            self.node.get_logger().warn(self._status_message)
            return None

        if future.exception() is not None:
            self._status_message = f"{service_name} failed: {future.exception()}"
            self.node.get_logger().warn(self._status_message)
            return None

        response = future.result()
        if response is None:
            self._status_message = f"{service_name} failed to return a response."
            self.node.get_logger().warn(self._status_message)
            return None

        return response
