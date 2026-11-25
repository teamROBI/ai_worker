#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class StereoRgbViewer(Node):
    def __init__(self):
        super().__init__('stereo_rgb_viewer')

        # Parameters (override via CLI)
        self.declare_parameter('left_topic', '/camera_left/camera_left/color/image_rect_raw')
        self.declare_parameter('right_topic', '/camera_right/camera_right/color/image_rect_raw')
        self.declare_parameter('window_name', 'Left | Right RGB (with resolution)')
        self.declare_parameter('queue_size', 10)

        self.left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        self.bridge = CvBridge()

        self.left_frame = None
        self.right_frame = None

        self.create_subscription(Image, self.left_topic, self.left_cb, self.queue_size)
        self.create_subscription(Image, self.right_topic, self.right_cb, self.queue_size)

        self.get_logger().info(f"Subscribing left : {self.left_topic}")
        self.get_logger().info(f"Subscribing right: {self.right_topic}")

        # Refresh UI at ~30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.render)

    def left_cb(self, msg: Image):
        self.left_frame = self._to_bgr(msg)

    def right_cb(self, msg: Image):
        self.right_frame = self._to_bgr(msg)

    def _to_bgr(self, msg: Image):
        # Try common encodings; fall back safely
        try:
            # If msg.encoding is "rgb8", this converts properly to BGR for OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            return cv_img
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return None

    @staticmethod
    def _annotate(img, label: str):
        h, w = img.shape[:2]
        text = f"{label}  {w}x{h}"
        # Draw readable text with outline
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    @staticmethod
    def _placeholder(title: str, w=640, h=480):
        img = np.zeros((h, w, 3), dtype=np.uint8)
        msg = f"Waiting for {title}..."
        cv2.putText(img, msg, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    def render(self):
        left = self.left_frame
        right = self.right_frame

        if left is None:
            left = self._placeholder("LEFT RGB")
        else:
            left = self._annotate(left.copy(), "LEFT")

        if right is None:
            right = self._placeholder("RIGHT RGB", w=left.shape[1], h=left.shape[0])
        else:
            right = self._annotate(right.copy(), "RIGHT")

        # Make heights match for side-by-side display
        lh, lw = left.shape[:2]
        rh, rw = right.shape[:2]
        target_h = max(lh, rh)

        def resize_to_h(img, target_h):
            h, w = img.shape[:2]
            if h == target_h:
                return img
            scale = target_h / float(h)
            new_w = max(1, int(w * scale))
            return cv2.resize(img, (new_w, target_h), interpolation=cv2.INTER_AREA)

        left_r = resize_to_h(left, target_h)
        right_r = resize_to_h(right, target_h)

        combined = np.hstack([left_r, right_r])

        cv2.imshow(self.window_name, combined)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quit requested (q). Shutting down.")
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = StereoRgbViewer()
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
