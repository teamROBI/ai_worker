#!/usr/bin/env python3
#
# VR Raw SG2 Publisher (Scenario 2 — Server Side)
#
# Runs a Vuer WebXR server and publishes raw VR controller/body data
# as ROS2 topics. Designed for the relay scenario where this node runs
# on a server PC and vr_raw_sg2_consumer runs on the robot.
#
# Quest 3 connects to this node over the external router network.
# ROS2 DDS delivers the raw topics to the consumer on the robot
# via the server's second NIC connected to the robot's AP.

import asyncio
import os
import socket
import threading

from geometry_msgs.msg import Point, PoseStamped, Quaternion
import nest_asyncio
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Joy
from vuer import Vuer
from vuer.schemas import Body, MotionControllers

nest_asyncio.apply()

BODY_HEAD_INDEX = 6
BODY_LEFT_ELBOW_INDEX = 10
BODY_RIGHT_ELBOW_INDEX = 15

VR_WORLD_TO_ROS_WORLD = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
], dtype=np.float64)


class VRRawSG2Publisher(Node):

    def __init__(self):
        super().__init__('vr_raw_sg2_publisher')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.declare_parameter('stream_fps', 30)
        self.declare_parameter('vuer_port', 8012)
        self.declare_parameter('cert_file', '')
        self.declare_parameter('key_file', '')

        self.fps = int(self.get_parameter('stream_fps').value)
        vuer_port = int(self.get_parameter('vuer_port').value)

        cert_param = str(self.get_parameter('cert_file').value)
        key_param = str(self.get_parameter('key_file').value)
        if not cert_param:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            cert_param = os.path.join(current_dir, 'cert.pem')
        if not key_param:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            key_param = os.path.join(current_dir, 'key.pem')

        hostname = socket.gethostbyname(socket.gethostname())
        ws_url = f'ws://{hostname}:{vuer_port}'

        self.vuer = Vuer(
            host='0.0.0.0',
            port=vuer_port,
            cert=cert_param,
            key=key_param,
            ws=ws_url,
            queries=dict(grid=False, reconnect=True),
            queue_len=3,
        )

        self.vuer.add_handler('BODY_MOVE')(self.on_body_tracking_move)
        self.vuer.add_handler('CONTROLLER_MOVE')(self.on_controller_move)

        vr_stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.head_pose_pub = self.create_publisher(
            PoseStamped, '/vr/raw/controller/head_pose_ros', vr_stream_qos)
        self.left_state_pub = self.create_publisher(
            Joy, '/vr/raw/controller/left_state', vr_stream_qos)
        self.right_state_pub = self.create_publisher(
            Joy, '/vr/raw/controller/right_state', vr_stream_qos)
        self.left_pose_pub = self.create_publisher(
            PoseStamped, '/vr/raw/controller/left_pose_ros', vr_stream_qos)
        self.right_pose_pub = self.create_publisher(
            PoseStamped, '/vr/raw/controller/right_pose_ros', vr_stream_qos)
        self.left_elbow_pub = self.create_publisher(
            PoseStamped, '/vr/raw/controller/left_elbow_pose_ros', vr_stream_qos)
        self.right_elbow_pub = self.create_publisher(
            PoseStamped, '/vr/raw/controller/right_elbow_pose_ros', vr_stream_qos)

        self.vr_world_to_ros_rot = R.from_matrix(VR_WORLD_TO_ROS_WORLD)

        self.log_counter = 0
        self.log_every_n = self.fps

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self._start_vuer_server()

        self.get_logger().info(f'VR Raw SG2 Publisher started at https://{hostname}:{vuer_port}')
        self.get_logger().info('Publishing raw VR data to /vr/raw/controller/* topics')

    def _start_vuer_server(self):
        def run_server():
            try:
                asyncio.set_event_loop(self.loop)
                self.vuer.spawn(start=True)(self.main_session)
            except Exception as e:
                self.get_logger().error(f'Vuer server error: {e}')

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    async def main_session(self, session):
        fps = self.fps
        self.get_logger().info('VR session started — enabling controllers + body tracking')
        session.upsert(
            MotionControllers(
                stream=True,
                key='motion-controller',
                left=True,
                right=True,
            ),
            to='bgChildren',
        )
        session.upsert(
            Body(
                fps=fps,
                stream=True,
                key='body_tracking',
                leftHand=False,
                rightHand=False,
                hideIndicate=False,
                showFrame=True,
                showBody=True,
                frameScale=0.02,
            ),
            to='children',
        )
        while True:
            await asyncio.sleep(1 / fps)

    # -- Helpers --------------------------------------------------------

    def _vr_matrix_to_ros_pose(self, flat_data):
        """Convert flat 16-element VR 4x4 matrix to PoseStamped in ros_world."""
        mat = np.asarray(flat_data, dtype=np.float64).reshape(4, 4, order='F')
        if not np.all(np.isfinite(mat)):
            return None
        if abs(float(np.linalg.det(mat[:3, :3]))) < 1e-6:
            return None

        pos_ros = VR_WORLD_TO_ROS_WORLD @ mat[:3, 3]
        rot_ros = self.vr_world_to_ros_rot * R.from_matrix(mat[:3, :3]) * self.vr_world_to_ros_rot.inv()
        quat = rot_ros.as_quat()  # [x, y, z, w]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'ros_world'
        pose.pose.position = Point(
            x=float(pos_ros[0]), y=float(pos_ros[1]), z=float(pos_ros[2]))
        pose.pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        return pose

    def _get_body_joint_flat(self, body_array, joint_index):
        start = joint_index * 16
        end = start + 16
        if body_array.size < end:
            return None
        return body_array[start:end]

    def _state_to_joy(self, state):
        """Convert Vuer controller state dict to sensor_msgs/Joy."""
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        tv = state.get('thumbstickValue', [0.0, 0.0])
        if not isinstance(tv, (list, tuple)) or len(tv) < 2:
            tv = [0.0, 0.0]
        joy.axes = [
            float(tv[0]),
            float(tv[1]),
            float(state.get('triggerValue', 0.0)),
            float(state.get('squeezeValue', 0.0)),
        ]
        joy.buttons = [
            int(bool(state.get('thumbstick', False))),
            int(bool(state.get('aButton', False))),
            int(bool(state.get('bButton', False))),
            int(bool(state.get('xButton', False))),
            int(bool(state.get('yButton', False))),
        ]
        return joy

    # -- Vuer event handlers --------------------------------------------

    async def on_body_tracking_move(self, event, session):
        try:
            if not isinstance(event.value, dict):
                return
            body_data = event.value.get('body')
            if not isinstance(body_data, (list, tuple, np.ndarray)):
                return
            body_array = (
                body_data if isinstance(body_data, np.ndarray)
                else np.asarray(body_data, dtype=np.float64)
            )

            head_flat = self._get_body_joint_flat(body_array, BODY_HEAD_INDEX)
            if head_flat is not None:
                pose = self._vr_matrix_to_ros_pose(head_flat)
                if pose is not None:
                    self.head_pose_pub.publish(pose)

            left_elbow_flat = self._get_body_joint_flat(body_array, BODY_LEFT_ELBOW_INDEX)
            if left_elbow_flat is not None:
                pose = self._vr_matrix_to_ros_pose(left_elbow_flat)
                if pose is not None:
                    self.left_elbow_pub.publish(pose)

            right_elbow_flat = self._get_body_joint_flat(body_array, BODY_RIGHT_ELBOW_INDEX)
            if right_elbow_flat is not None:
                pose = self._vr_matrix_to_ros_pose(right_elbow_flat)
                if pose is not None:
                    self.right_elbow_pub.publish(pose)

        except Exception as e:
            self.get_logger().error(f'Body tracking error: {e}')

    async def on_controller_move(self, event, session):
        try:
            if not isinstance(event.value, dict):
                return
            data = event.value

            left_state = data.get('leftState')
            if isinstance(left_state, dict):
                self.left_state_pub.publish(self._state_to_joy(left_state))

            right_state = data.get('rightState')
            if isinstance(right_state, dict):
                self.right_state_pub.publish(self._state_to_joy(right_state))

            left_raw = data.get('left')
            if isinstance(left_raw, (list, np.ndarray)) and len(left_raw) == 16:
                pose = self._vr_matrix_to_ros_pose(left_raw)
                if pose is not None:
                    self.left_pose_pub.publish(pose)

            right_raw = data.get('right')
            if isinstance(right_raw, (list, np.ndarray)) and len(right_raw) == 16:
                pose = self._vr_matrix_to_ros_pose(right_raw)
                if pose is not None:
                    self.right_pose_pub.publish(pose)

            self.log_counter += 1
            if self.log_counter % self.log_every_n == 0:
                self.get_logger().info(
                    f'Controller | L_state={left_state is not None} '
                    f'R_state={right_state is not None} '
                    f'L_pose={left_raw is not None} '
                    f'R_pose={right_raw is not None}'
                )

        except Exception as e:
            self.get_logger().error(f'Controller move error: {e}')

    def __del__(self):
        try:
            if hasattr(self, 'vuer'):
                self.loop.run_until_complete(self.vuer.stop())
            if hasattr(self, 'loop'):
                self.loop.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VRRawSG2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
