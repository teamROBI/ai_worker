#!/usr/bin/env python3
"""
Robot Camera Recorder (headless)
Records head (ZED) and wrist (RealSense) cameras without a display.
Press Enter or Ctrl+C to stop recording.

Usage:
    python3 record_cameras_headless.py [recording_name]
    python3 record_cameras_headless.py my_episode
    python3 record_cameras_headless.py --ros-args -p head_topic:=/zed/...
"""

import argparse
import os
import subprocess
import sys
import threading
import time
from datetime import datetime, timedelta
from io import BytesIO
from pathlib import Path
from typing import BinaryIO

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# ── Constants ────────────────────────────────────────────────────────────────

RECORD_FPS = 30.0
VIDEOS_DIR = Path(__file__).parent / "videos"

CAMERAS = ["head", "left", "right"]

DEFAULT_TOPICS = {
    "head":  "/zed/zed_node/left/image_rect_color/compressed",
    "left":  "/camera_left/camera_left/color/image_rect_raw/compressed",
    "right": "/camera_right/camera_right/color/image_rect_raw/compressed",
}

# ── ROS2 Node ────────────────────────────────────────────────────────────────

class CameraRecorderNode(Node):
    def __init__(self):
        super().__init__("camera_recorder_headless")

        for cam in CAMERAS:
            self.declare_parameter(f"{cam}_topic", DEFAULT_TOPICS[cam])

        topics = {cam: self.get_parameter(f"{cam}_topic").get_parameter_value().string_value
                  for cam in CAMERAS}

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._lock = threading.Lock()
        self._frames: dict = {cam: None for cam in CAMERAS}

        self._writers_lock = threading.Lock()
        self._record_writers: dict[str, BinaryIO] = {}
        self._record_counts: dict[str, int] = {}
        self._frame_dims: dict[str, tuple[int, int]] = {}  # cam -> (w, h)
        self._record_start: float | None = None

        for cam in CAMERAS:
            self.create_subscription(
                CompressedImage, topics[cam],
                lambda msg, c=cam: self._cb(msg, c),
                image_qos,
            )
            self.get_logger().info(f"Subscribed [{cam}]: {topics[cam]}")

    def _cb(self, msg: CompressedImage, cam: str):
        try:
            img = PILImage.open(BytesIO(bytes(msg.data))).convert("RGB")
            frame = np.array(img, dtype=np.uint8)[:, :, ::-1].copy()  # RGB→BGR
            with self._lock:
                self._frames[cam] = frame
            with self._writers_lock:
                if cam in self._record_writers:
                    self._record_writers[cam].write(frame.tobytes())
                    self._record_counts[cam] += 1
                    if cam not in self._frame_dims:
                        self._frame_dims[cam] = (frame.shape[1], frame.shape[0])  # w, h
        except Exception as e:
            self.get_logger().warn(f"[{cam}] decode failed: {e}")

    def get_frames(self) -> dict:
        with self._lock:
            return {cam: (f.copy() if f is not None else None)
                    for cam, f in self._frames.items()}

    def start_recording(self, writers: dict[str, BinaryIO]):
        with self._writers_lock:
            self._record_writers = writers
            self._record_counts = {cam: 0 for cam in writers}
            self._frame_dims = {}
            self._record_start = time.monotonic()

    def stop_recording(self):
        with self._writers_lock:
            counts = dict(self._record_counts)
            elapsed = time.monotonic() - self._record_start if self._record_start else 0
            dims = dict(self._frame_dims)
            self._record_writers = {}
            self._record_counts = {}
            self._frame_dims = {}
            self._record_start = None
        return counts, elapsed, dims

    def get_record_counts(self) -> dict:
        with self._writers_lock:
            return dict(self._record_counts)


# ── Headless recorder ────────────────────────────────────────────────────────

class HeadlessRecorder:
    def __init__(self, node: CameraRecorderNode, rec_name: str,
                 cameras: list[str] | None = None):
        self.node = node
        self.rec_name = rec_name.strip().replace(" ", "_")
        self.cameras = cameras or CAMERAS
        self._raw_writers: dict[str, BinaryIO] = {}
        self._raw_paths: dict[str, Path] = {}
        self._stop_event = threading.Event()

    def run(self):
        VIDEOS_DIR.mkdir(parents=True, exist_ok=True)

        if not self._wait_for_cameras():
            return

        if not self._start_recording():
            return

        self._monitor()
        self._stop_recording()

    # ── Internal ─────────────────────────────────────────────────────────────

    def _wait_for_cameras(self) -> bool:
        print("Waiting for cameras", end="", flush=True)
        timeout = 10.0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                print("\nAborted.")
                return False
            frames = self.node.get_frames()
            ready = [c for c in self.cameras if frames[c] is not None]
            if len(ready) == len(self.cameras):
                print(f"\nAll cameras ready: {ready}")
                return True
            print(".", end="", flush=True)
            time.sleep(0.5)

        frames = self.node.get_frames()
        missing = [c for c in self.cameras if frames[c] is None]
        print(f"\nERROR: cameras not responding: {missing}")
        print("Please restart the robot and try again.")
        return False

    def _start_recording(self) -> bool:
        frames = self.node.get_frames()
        rec_dir = VIDEOS_DIR / self.rec_name
        rec_dir.mkdir(parents=True, exist_ok=True)

        for cam in self.cameras:
            if frames.get(cam) is None:
                continue
            raw_path = rec_dir / f"{cam}.raw"
            try:
                f = open(raw_path, "wb")
            except OSError as e:
                print(f"ERROR: could not open raw file for {cam}: {e}")
                for w in self._raw_writers.values():
                    w.close()
                return False
            self._raw_writers[cam] = f
            self._raw_paths[cam] = raw_path

        if not self._raw_writers:
            print("ERROR: no cameras available to record.")
            return False

        self.node.start_recording(self._raw_writers)
        print(f"\nRecording  →  {VIDEOS_DIR / self.rec_name}")
        print(f"Cameras    :  {list(self._raw_writers.keys())}")
        print("Press Enter or Ctrl+C to stop.\n")
        return True

    def _monitor(self):
        t = threading.Thread(target=self._wait_for_enter, daemon=True)
        t.start()

        start = time.monotonic()
        try:
            while not self._stop_event.is_set():
                elapsed = time.monotonic() - start
                td = timedelta(seconds=int(elapsed))
                counts = self.node.get_record_counts()
                cam_info = "  ".join(f"{c}:{counts.get(c, 0)}" for c in self._raw_writers)
                print(f"\r  {td}  |  {cam_info}  ", end="", flush=True)
                time.sleep(0.5)
        except KeyboardInterrupt:
            self._stop_event.set()
        print()

    def _wait_for_enter(self):
        try:
            sys.stdin.readline()
        except Exception:
            pass
        self._stop_event.set()

    def _stop_recording(self):
        counts, elapsed, dims = self.node.stop_recording()

        for f in self._raw_writers.values():
            f.close()
        self._raw_writers.clear()

        fps_map = {
            cam: max(1.0, counts.get(cam, 1) / elapsed) if elapsed > 0 else RECORD_FPS
            for cam in self._raw_paths
        }

        total = sum(counts.values())
        print(f"Stopped — {total} frames, {elapsed:.1f}s")
        for cam in self._raw_paths:
            print(f"  {cam}: {counts.get(cam, 0)} frames @ {fps_map.get(cam, 0):.2f} fps  "
                  f"dim={dims.get(cam, '?')}")

        if total == 0:
            print("No frames recorded — skipping encode.")
            for p in self._raw_paths.values():
                p.unlink(missing_ok=True)
            return

        print("Re-encoding to H.264…")
        self._reencode_all(fps_map, dims)

    def _reencode_all(self, fps_map: dict, dims: dict):
        for cam, raw_path in self._raw_paths.items():
            final_path = VIDEOS_DIR / self.rec_name / f"{cam}.mp4"
            fps = fps_map.get(cam, RECORD_FPS)
            wh = dims.get(cam)
            if wh is None:
                print(f"  SKIP {cam}: no frame dimensions recorded.")
                raw_path.unlink(missing_ok=True)
                continue
            w, h = wh
            print(f"  ffmpeg: {raw_path.name} → {final_path.name}  "
                  f"({w}x{h} @ {fps:.2f} fps)", flush=True)
            try:
                subprocess.run(
                    ["ffmpeg", "-y",
                     "-f", "rawvideo",
                     "-vcodec", "rawvideo",
                     "-s", f"{w}x{h}",
                     "-pix_fmt", "bgr24",
                     "-r", f"{fps:.6f}",
                     "-i", str(raw_path),
                     "-vcodec", "libx264",
                     "-pix_fmt", "yuv420p",
                     str(final_path)],
                    check=True, capture_output=True,
                )
                raw_path.unlink(missing_ok=True)
            except subprocess.CalledProcessError as e:
                print(f"  ERROR re-encoding {cam}: {e.stderr.decode()}")
        print(f"Saved → {VIDEOS_DIR / self.rec_name}")


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("rec_name", nargs="?",
                        default=datetime.now().strftime("recording_%Y%m%d_%H%M%S"))
    parser.add_argument("--cameras", nargs="+", choices=CAMERAS, default=None)
    known, _ = parser.parse_known_args()

    rclpy.init()
    node = CameraRecorderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    recorder = HeadlessRecorder(node, known.rec_name, known.cameras)
    try:
        recorder.run()
    except KeyboardInterrupt:
        pass
    finally:
        os._exit(0)


if __name__ == "__main__":
    main()
