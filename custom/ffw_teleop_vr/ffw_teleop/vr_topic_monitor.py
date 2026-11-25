#!/usr/bin/env python3
"""
VR Topic Monitor GUI — real-time display of all VR controller topics.
Shows Hz rates, button states, axis values, and pose positions.
"""

import math
import threading
import time
from collections import deque

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float32
import tkinter as tk
from trajectory_msgs.msg import JointTrajectory


class TopicStats:
    def __init__(self):
        self.timestamps = deque(maxlen=100)
        self.last_msg = None

    def update(self, msg):
        self.timestamps.append(time.time())
        self.last_msg = msg

    @property
    def hz(self):
        if len(self.timestamps) < 2:
            return 0.0
        dt = self.timestamps[-1] - self.timestamps[0]
        if dt < 0.001:
            return 0.0
        return (len(self.timestamps) - 1) / dt


class VRTopicMonitor(Node):
    def __init__(self):
        super().__init__('vr_topic_monitor')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.stats = {}

        # Raw VR topics
        raw_pose_topics = [
            '/vr/raw/controller/head_pose_ros',
            '/vr/raw/controller/left_pose_ros',
            '/vr/raw/controller/right_pose_ros',
            '/vr/raw/controller/left_elbow_pose_ros',
            '/vr/raw/controller/right_elbow_pose_ros',
        ]
        raw_joy_topics = [
            '/vr/raw/controller/left_state',
            '/vr/raw/controller/right_state',
        ]

        # Robot command topics
        cmd_pose_topics = [
            '/l_goal_pose',
            '/r_goal_pose',
            '/l_elbow_pose',
            '/r_elbow_pose',
        ]
        float_topics = [
            '/vr_controller/left_trigger',
            '/vr_controller/right_trigger',
            '/vr_controller/left_squeeze',
            '/vr_controller/right_squeeze',
        ]
        joint_traj_topics = [
            '/leader/joystick_controller_left/joint_trajectory',
        ]

        for topic in raw_pose_topics + cmd_pose_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(PoseStamped, topic,
                                     lambda msg, t=topic: self.stats[t].update(msg), qos)

        for topic in raw_joy_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(Joy, topic,
                                     lambda msg, t=topic: self.stats[t].update(msg), qos)

        for topic in float_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(Float32, topic,
                                     lambda msg, t=topic: self.stats[t].update(msg), qos)

        for topic in joint_traj_topics:
            self.stats[topic] = TopicStats()
            self.create_subscription(JointTrajectory, topic,
                                     lambda msg, t=topic: self.stats[t].update(msg), 10)

        self.stats['/joint_states'] = TopicStats()
        self.create_subscription(JointState, '/joint_states',
                                 lambda msg: self.stats['/joint_states'].update(msg), 10)


def quat_to_rpy(q):
    """Return (roll, pitch, yaw) in degrees from geometry_msgs quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def format_pose(msg, show_orientation=False):
    if msg is None:
        return '---'
    p = msg.pose.position
    pos_str = f'x:{p.x:+.3f}  y:{p.y:+.3f}  z:{p.z:+.3f}'
    if not show_orientation:
        return pos_str
    _, pitch, yaw = quat_to_rpy(msg.pose.orientation)
    return f'{pos_str}\ntilt:{pitch:+.1f}°  pan:{yaw:+.1f}°'


def format_joy(msg, side='left'):
    if msg is None:
        return 'axes: ---\nbtn:  ---'
    labels = ['thX', 'thY', 'trig', 'sqz']
    if side == 'left':
        btn_labels = ['thBtn', 'X', 'Y']
    else:
        btn_labels = ['thBtn', 'A', 'B']
    ax_named = '  '.join(f'{labels[i]}:{msg.axes[i]:+.2f}' for i in range(min(len(msg.axes), len(labels))))
    bt_named = '  '.join(f'{btn_labels[i]}:{int(msg.buttons[i])}' for i in range(min(len(msg.buttons), len(btn_labels))))
    return f'{ax_named}\n{bt_named}'


def format_float(msg):
    if msg is None:
        return '---'
    return f'{msg.data:.3f}'


HEAD_JOINT_NAMES = {'head_joint1': 'tilt', 'head_joint2': 'pan'}


def format_joint_trajectory(msg):
    if msg is None or not msg.joint_names or not msg.points:
        return '---'
    positions = msg.points[0].positions
    parts = []
    for name, pos in zip(msg.joint_names, positions):
        label = HEAD_JOINT_NAMES.get(name, name)
        parts.append(f'{label}:{math.degrees(pos):+.1f}°')
    return '  '.join(parts)


def format_joint_state(msg):
    if msg is None:
        return '---'
    interesting = ['head_joint1', 'head_joint2', 'lift_joint']
    parts = []
    for name in interesting:
        if name in msg.name:
            idx = msg.name.index(name)
            val = msg.position[idx]
            label = HEAD_JOINT_NAMES.get(name, name)
            if name.startswith('head'):
                parts.append(f'{label}:{math.degrees(val):+.1f}°')
            else:
                parts.append(f'{label}:{val:+.4f}')
    return '  '.join(parts) if parts else '---'


def hz_color(hz):
    if hz > 20:
        return '#00cc00'
    elif hz > 5:
        return '#cccc00'
    else:
        return '#cc0000'


def build_gui(node):
    root = tk.Tk()
    root.title('VR Topic Monitor')
    root.configure(bg='#1e1e1e')

    labels = {}

    style = {'bg': '#1e1e1e', 'fg': '#ffffff', 'font': ('Consolas', 10)}
    header_style = {'bg': '#1e1e1e', 'fg': '#00aaff', 'font': ('Consolas', 11, 'bold')}
    hz_style = {'bg': '#1e1e1e', 'fg': '#00cc00', 'font': ('Consolas', 10, 'bold')}
    val_style = {'bg': '#1e1e1e', 'fg': '#cccccc', 'font': ('Consolas', 9)}

    row = 0

    def add_section(title):
        nonlocal row
        tk.Label(root, text=f'━━━ {title} ━━━', **header_style).grid(
            row=row, column=0, columnspan=3, sticky='w', padx=5, pady=(10, 2))
        row += 1

    def add_topic(topic, short_name):
        nonlocal row
        tk.Label(root, text=short_name, **style).grid(row=row, column=0, sticky='w', padx=5)
        hz_lbl = tk.Label(root, text='0.0 Hz', **hz_style, width=10)
        hz_lbl.grid(row=row, column=1, sticky='w', padx=5)
        val_lbl = tk.Label(root, text='---', **val_style, anchor='w', justify='left')
        val_lbl.grid(row=row, column=2, sticky='w', padx=5)
        labels[topic] = (hz_lbl, val_lbl)
        row += 1

    # Raw VR topics
    add_section('RAW VR DATA (from server)')
    add_topic('/vr/raw/controller/head_pose_ros', 'Head Pose')
    add_topic('/vr/raw/controller/left_pose_ros', 'Left Ctrl Pose')
    add_topic('/vr/raw/controller/right_pose_ros', 'Right Ctrl Pose')
    add_topic('/vr/raw/controller/left_elbow_pose_ros', 'Left Elbow')
    add_topic('/vr/raw/controller/right_elbow_pose_ros', 'Right Elbow')
    add_topic('/vr/raw/controller/left_state', 'Left State')
    add_topic('/vr/raw/controller/right_state', 'Right State')

    # Robot command topics
    add_section('ROBOT COMMANDS (from consumer)')
    add_topic('/l_goal_pose', 'L Goal Pose')
    add_topic('/r_goal_pose', 'R Goal Pose')
    add_topic('/l_elbow_pose', 'L Elbow Pose')
    add_topic('/r_elbow_pose', 'R Elbow Pose')
    add_topic('/vr_controller/left_trigger', 'L Trigger')
    add_topic('/vr_controller/right_trigger', 'R Trigger')
    add_topic('/vr_controller/left_squeeze', 'L Squeeze')
    add_topic('/vr_controller/right_squeeze', 'R Squeeze')
    add_topic('/leader/joystick_controller_left/joint_trajectory', 'Head Pan/Tilt Cmd')

    # Robot state topics
    add_section('ROBOT STATE')
    add_topic('/joint_states', 'Joint States')

    def update():
        for topic, (hz_lbl, val_lbl) in labels.items():
            if topic not in node.stats:
                continue
            s = node.stats[topic]
            hz = s.hz
            hz_lbl.config(text=f'{hz:5.1f} Hz', fg=hz_color(hz))

            msg = s.last_msg
            if isinstance(msg, PoseStamped):
                show_orient = topic == '/vr/raw/controller/head_pose_ros'
                val_lbl.config(text=format_pose(msg, show_orientation=show_orient))
            elif isinstance(msg, Joy):
                side = 'left' if 'left' in topic else 'right'
                val_lbl.config(text=format_joy(msg, side))
            elif isinstance(msg, Float32):
                val_lbl.config(text=format_float(msg))
            elif isinstance(msg, JointTrajectory):
                val_lbl.config(text=format_joint_trajectory(msg))
            elif isinstance(msg, JointState):
                val_lbl.config(text=format_joint_state(msg))

        root.after(100, update)

    root.bind('<Escape>', lambda e: root.destroy())

    update()
    return root


def main(args=None):
    rclpy.init(args=args)
    node = VRTopicMonitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = build_gui(node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
