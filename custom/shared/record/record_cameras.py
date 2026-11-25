#!/usr/bin/env python3
"""
Robot Camera Recorder
Records head (ZED) and wrist (RealSense) cameras to MP4 files with a Tkinter GUI.

Usage:
    python3 record_cameras.py
    python3 record_cameras.py --ros-args -p head_topic:=/zed/zed_node/left/image_rect_color/compressed
"""

import os
import re
import subprocess
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

import tkinter as tk
from tkinter import messagebox, ttk

try:
    from PIL import Image as PILImage, ImageTk
except ImportError:
    raise SystemExit("Pillow is required: pip install Pillow")

# ── Constants ────────────────────────────────────────────────────────────────

RECORD_FPS = 30.0
TICK_MS = int(1000 / RECORD_FPS)           # ~33 ms
PREVIEW_W, PREVIEW_H = 320, 240
VIDEOS_DIR = Path(__file__).parent / "videos"

CAMERAS = ["head", "left", "right"]

DEFAULT_TOPICS = {
    "head":  "/zed/zed_node/left/image_rect_color/compressed",
    "left":  "/camera_left/camera_left/color/image_rect_raw/compressed",
    "right": "/camera_right/camera_right/color/image_rect_raw/compressed",
}

COLORS = {
    "bg":        "#1e1e2e",
    "panel":     "#2a2a3d",
    "accent":    "#7c3aed",
    "rec":       "#dc2626",
    "idle":      "#6b7280",
    "green":     "#22c55e",
    "text":      "#e2e8f0",
    "subtext":   "#94a3b8",
    "border":    "#374151",
    "entry_bg":  "#111827",
    "hover":     "#6d28d9",
}

# ── ROS2 Node ────────────────────────────────────────────────────────────────

class CameraRecorderNode(Node):
    def __init__(self):
        super().__init__("camera_recorder")

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
        self._frames = {cam: None for cam in CAMERAS}

        self._writers_lock = threading.Lock()
        self._record_writers: dict = {}
        self._record_counts: dict = {}
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
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with self._lock:
                    self._frames[cam] = frame
                with self._writers_lock:
                    if cam in self._record_writers:
                        self._record_writers[cam].write(frame)
                        self._record_counts[cam] += 1
        except Exception as e:
            self.get_logger().warn(f"[{cam}] decode failed: {e}")

    def get_frames(self) -> dict:
        with self._lock:
            return {cam: (f.copy() if f is not None else None)
                    for cam, f in self._frames.items()}

    def start_recording(self, writers: dict):
        with self._writers_lock:
            self._record_writers = writers
            self._record_counts = {cam: 0 for cam in writers}
            self._record_start = time.monotonic()

    def stop_recording(self):
        with self._writers_lock:
            counts = dict(self._record_counts)
            elapsed = time.monotonic() - self._record_start if self._record_start else 0
            self._record_writers = {}
            self._record_counts = {}
            self._record_start = None
        return counts, elapsed

    def get_record_counts(self) -> dict:
        with self._writers_lock:
            return dict(self._record_counts)


# ── GUI ──────────────────────────────────────────────────────────────────────

class RecorderGUI:
    def __init__(self, node: CameraRecorderNode):
        self.node = node

        # recording state
        self._recording = False
        self._writers: dict[str, cv2.VideoWriter] = {}
        self._tmp_paths: dict[str, Path] = {}
        self._rec_name: str = ""
        self._rec_start: float | None = None

        # per-camera enable toggles
        self._enabled: dict[str, tk.BooleanVar] = {}

        VIDEOS_DIR.mkdir(parents=True, exist_ok=True)

        self._build_window()
        self._refresh_recordings()
        self._tick()
        self.root.mainloop()

    # ── Window construction ──────────────────────────────────────────────────

    def _build_window(self):
        self.root = tk.Tk()
        self.root.title("Robot Camera Recorder")
        self.root.configure(bg=COLORS["bg"])
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_title()
        self._build_previews()
        self._build_controls()
        self._build_status()
        self._build_recordings()

    def _build_title(self):
        bar = tk.Frame(self.root, bg=COLORS["panel"], pady=10)
        bar.pack(fill="x")
        tk.Label(
            bar, text="  Robot Camera Recorder",
            font=("Helvetica", 16, "bold"),
            bg=COLORS["panel"], fg=COLORS["text"],
        ).pack(side="left", padx=14)

    def _build_previews(self):
        frame = tk.Frame(self.root, bg=COLORS["bg"], pady=8)
        frame.pack(fill="x", padx=10)

        self._preview_labels: dict[str, tk.Label] = {}
        self._photo_refs: dict[str, object] = {}

        for cam in CAMERAS:
            col = tk.Frame(frame, bg=COLORS["panel"], bd=0,
                           highlightthickness=1, highlightbackground=COLORS["border"],
                           padx=4, pady=4)
            col.pack(side="left", expand=True, fill="both", padx=4)

            # Camera label header
            hdr = tk.Frame(col, bg=COLORS["panel"])
            hdr.pack(fill="x")
            tk.Label(
                hdr, text=cam.upper(),
                font=("Helvetica", 10, "bold"),
                bg=COLORS["panel"], fg=COLORS["accent"],
            ).pack(side="left")

            var = tk.BooleanVar(value=True)
            self._enabled[cam] = var
            tk.Checkbutton(
                hdr, text="Record", variable=var,
                bg=COLORS["panel"], fg=COLORS["subtext"],
                selectcolor=COLORS["entry_bg"],
                activebackground=COLORS["panel"],
                font=("Helvetica", 9),
            ).pack(side="right")

            # Placeholder canvas for preview
            lbl = tk.Label(
                col,
                bg="#0a0a14",
                text=f"Waiting for\n{cam} camera…",
                fg=COLORS["subtext"],
                font=("Helvetica", 9),
                width=PREVIEW_W // 8,
                height=PREVIEW_H // 16,
            )
            lbl.pack()
            self._preview_labels[cam] = lbl

    def _build_controls(self):
        outer = tk.Frame(self.root, bg=COLORS["bg"], padx=10, pady=4)
        outer.pack(fill="x")

        panel = tk.Frame(outer, bg=COLORS["panel"],
                         highlightthickness=1, highlightbackground=COLORS["border"],
                         padx=12, pady=10)
        panel.pack(fill="x")

        # Row 1: recording name
        row1 = tk.Frame(panel, bg=COLORS["panel"])
        row1.pack(fill="x", pady=(0, 6))

        tk.Label(row1, text="Recording name:", bg=COLORS["panel"],
                 fg=COLORS["subtext"], font=("Helvetica", 10)).pack(side="left")

        self._name_var = tk.StringVar(value=self._default_name())
        entry = tk.Entry(
            row1, textvariable=self._name_var,
            bg=COLORS["entry_bg"], fg=COLORS["text"],
            insertbackground=COLORS["text"],
            relief="flat", font=("Helvetica", 10),
            width=28,
        )
        entry.pack(side="left", padx=(8, 4))

        tk.Button(
            row1, text="Now", command=self._set_timestamp_name,
            bg=COLORS["border"], fg=COLORS["text"],
            relief="flat", font=("Helvetica", 9),
            padx=6, pady=2, cursor="hand2",
        ).pack(side="left")

        # Row 2: output folder
        row2 = tk.Frame(panel, bg=COLORS["panel"])
        row2.pack(fill="x")

        tk.Label(row2, text="Output folder:", bg=COLORS["panel"],
                 fg=COLORS["subtext"], font=("Helvetica", 10)).pack(side="left")
        tk.Label(row2, text=str(VIDEOS_DIR), bg=COLORS["panel"],
                 fg=COLORS["text"], font=("Helvetica", 9)).pack(side="left", padx=8)

    def _build_status(self):
        outer = tk.Frame(self.root, bg=COLORS["bg"], padx=10, pady=4)
        outer.pack(fill="x")

        panel = tk.Frame(outer, bg=COLORS["panel"],
                         highlightthickness=1, highlightbackground=COLORS["border"],
                         padx=12, pady=10)
        panel.pack(fill="x")

        row1 = tk.Frame(panel, bg=COLORS["panel"])
        row1.pack(fill="x")

        self._rec_btn = tk.Button(
            row1, text="  START RECORDING",
            command=self._toggle_recording,
            bg=COLORS["accent"], fg="white",
            relief="flat", font=("Helvetica", 11, "bold"),
            padx=18, pady=8, cursor="hand2",
        )
        self._rec_btn.pack(side="left")

        self._status_dot = tk.Label(
            row1, text="●", fg=COLORS["idle"],
            bg=COLORS["panel"], font=("Helvetica", 14),
        )
        self._status_dot.pack(side="left", padx=(14, 2))

        self._status_lbl = tk.Label(
            row1, text="IDLE",
            bg=COLORS["panel"], fg=COLORS["idle"],
            font=("Helvetica", 11, "bold"),
        )
        self._status_lbl.pack(side="left")

        row2 = tk.Frame(panel, bg=COLORS["panel"])
        row2.pack(fill="x", pady=(6, 0))

        self._duration_lbl = tk.Label(
            row2, text="Duration:  --:--:--",
            bg=COLORS["panel"], fg=COLORS["subtext"], font=("Helvetica", 10),
        )
        self._duration_lbl.pack(side="left")

        self._frames_lbl = tk.Label(
            row2, text="    Frames: 0",
            bg=COLORS["panel"], fg=COLORS["subtext"], font=("Helvetica", 10),
        )
        self._frames_lbl.pack(side="left")

        # Per-camera frame count
        self._cam_frame_lbls: dict[str, tk.Label] = {}
        for cam in CAMERAS:
            lbl = tk.Label(
                row2, text=f"    {cam}: 0",
                bg=COLORS["panel"], fg=COLORS["subtext"], font=("Helvetica", 9),
            )
            lbl.pack(side="left")
            self._cam_frame_lbls[cam] = lbl

    def _build_recordings(self):
        outer = tk.Frame(self.root, bg=COLORS["bg"], padx=10, pady=4)
        outer.pack(fill="both", expand=True)

        hdr = tk.Frame(outer, bg=COLORS["bg"])
        hdr.pack(fill="x", pady=(0, 4))
        tk.Label(hdr, text="Saved Recordings", bg=COLORS["bg"],
                 fg=COLORS["text"], font=("Helvetica", 11, "bold")).pack(side="left")

        # Listbox with scrollbar
        list_frame = tk.Frame(outer, bg=COLORS["panel"],
                              highlightthickness=1, highlightbackground=COLORS["border"])
        list_frame.pack(fill="both", expand=True)

        scrollbar = tk.Scrollbar(list_frame, bg=COLORS["border"])
        scrollbar.pack(side="right", fill="y")

        self._rec_list = tk.Listbox(
            list_frame, yscrollcommand=scrollbar.set,
            bg=COLORS["entry_bg"], fg=COLORS["text"],
            selectbackground=COLORS["accent"],
            font=("Courier", 9),
            relief="flat", borderwidth=0,
            height=8,
        )
        self._rec_list.pack(fill="both", expand=True)
        scrollbar.config(command=self._rec_list.yview)

        # Action buttons
        btn_row = tk.Frame(outer, bg=COLORS["bg"], pady=4)
        btn_row.pack(fill="x")

        self._btn_style = {"relief": "flat", "font": ("Helvetica", 9),
                           "padx": 10, "pady": 4, "cursor": "hand2"}

        tk.Button(btn_row, text="Open Folder", command=self._open_folder,
                  bg=COLORS["border"], fg=COLORS["text"], **self._btn_style).pack(side="left", padx=(0, 4))

        tk.Button(btn_row, text="Refresh", command=self._refresh_recordings,
                  bg=COLORS["border"], fg=COLORS["text"], **self._btn_style).pack(side="left", padx=(0, 4))

        tk.Button(btn_row, text="Delete Selected", command=self._delete_selected,
                  bg="#7f1d1d", fg="white", **self._btn_style).pack(side="left")

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _default_name() -> str:
        return datetime.now().strftime("recording_%Y%m%d_%H%M%S")

    def _set_timestamp_name(self):
        self._name_var.set(datetime.now().strftime("recording_%Y%m%d_%H%M%S"))

    def _validate_name(self, name: str) -> str | None:
        name = name.strip()
        if not name:
            return "Recording name cannot be empty."
        if not re.match(r"^[\w\-. ]+$", name):
            return "Name may only contain letters, digits, spaces, -, _, ."
        return None

    def _recording_path(self, name: str, cam: str) -> Path:
        safe = name.strip().replace(" ", "_")
        return VIDEOS_DIR / safe / f"{cam}.mp4"

    # ── Recording control ────────────────────────────────────────────────────

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        name = self._name_var.get()
        err = self._validate_name(name)
        if err:
            messagebox.showerror("Invalid name", err)
            return

        enabled_cams = [c for c in CAMERAS if self._enabled[c].get()]
        if not enabled_cams:
            messagebox.showerror("No cameras", "Enable at least one camera to record.")
            return

        frames = self.node.get_frames()

        # Validate all cameras before creating any files
        for cam in enabled_cams:
            if frames[cam] is None:
                messagebox.showerror(
                    "Camera not ready",
                    f"No frame received from [{cam}] camera yet.\n"
                    "Wait until the preview appears before recording."
                )
                return

        rec_dir = VIDEOS_DIR / name.strip().replace(" ", "_")

        for cam in enabled_cams:
            path = self._recording_path(name, cam)
            if path.exists():
                if not messagebox.askyesno(
                    "File exists",
                    f"{path.relative_to(VIDEOS_DIR)} already exists. Overwrite?"
                ):
                    return

        rec_dir.mkdir(parents=True, exist_ok=True)

        writers = {}
        tmp_paths = {}
        for cam in enabled_cams:
            frame = frames[cam]
            tmp_path = self._recording_path(name, cam).with_name(f"{cam}_tmp.mp4")
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(str(tmp_path), fourcc, RECORD_FPS, (w, h))
            if not writer.isOpened():
                messagebox.showerror("Error", f"Could not open VideoWriter for {tmp_path}")
                for w_ in writers.values():
                    w_.release()
                return
            writers[cam] = writer
            tmp_paths[cam] = tmp_path

        self._writers = writers
        self._tmp_paths = tmp_paths
        self._rec_name = name
        self._rec_start = time.monotonic()
        self._recording = True
        self.node.start_recording(writers)

        self._rec_btn.config(text="  STOP RECORDING", bg=COLORS["rec"])
        self._status_dot.config(fg=COLORS["rec"])
        self._status_lbl.config(text="RECORDING", fg=COLORS["rec"])
        self.node.get_logger().info(f"Recording started: {name}")

    def _stop_recording(self):
        self._recording = False
        counts, elapsed = self.node.stop_recording()

        for writer in self._writers.values():
            writer.release()
        self._writers.clear()
        self._rec_start = None

        fps_map = {
            cam: max(1.0, counts.get(cam, 1) / elapsed) if elapsed > 0 else RECORD_FPS
            for cam in self._tmp_paths
        }

        self._rec_btn.config(text="  START RECORDING", bg=COLORS["accent"])
        self._status_dot.config(fg=COLORS["green"])
        self._status_lbl.config(text="ENCODING…", fg=COLORS["green"])
        self._duration_lbl.config(text="Duration:  --:--:--")
        self._frames_lbl.config(text="    Frames: 0")
        for lbl in self._cam_frame_lbls.values():
            lbl.config(text="")

        total = sum(counts.values())
        self.node.get_logger().info(
            f"Recording stopped — {total} frames, {elapsed:.1f}s "
            f"({', '.join(f'{c}:{counts.get(c,0)}/{fps_map.get(c,0):.1f}fps' for c in self._tmp_paths)})"
        )

        tmp_paths = dict(self._tmp_paths)
        rec_name = self._rec_name
        self._tmp_paths.clear()
        threading.Thread(
            target=self._reencode_all, args=(rec_name, tmp_paths, fps_map), daemon=True
        ).start()

    def _reencode_all(self, rec_name: str, tmp_paths: dict, fps_map: dict):
        for cam, tmp_path in tmp_paths.items():
            final_path = self._recording_path(rec_name, cam)
            fps = fps_map.get(cam, RECORD_FPS)
            try:
                subprocess.run(
                    ["ffmpeg", "-y",
                     "-r", f"{fps:.6f}", "-i", str(tmp_path),
                     "-vcodec", "libx264", "-pix_fmt", "yuv420p",
                     str(final_path)],
                    check=True, capture_output=True,
                )
                tmp_path.unlink(missing_ok=True)
            except Exception as e:
                self.node.get_logger().error(f"ffmpeg re-encode failed [{cam}]: {e}")
        self.root.after(0, self._after_encode)

    def _after_encode(self):
        self._status_dot.config(fg=COLORS["idle"])
        self._status_lbl.config(text="IDLE", fg=COLORS["idle"])
        self._refresh_recordings()

    # ── Tick loop ────────────────────────────────────────────────────────────

    def _tick(self):
        frames = self.node.get_frames()

        # Update recording status display
        if self._recording:
            elapsed = time.monotonic() - self._rec_start
            td = timedelta(seconds=int(elapsed))
            counts = self.node.get_record_counts()
            total = sum(counts.values())
            self._duration_lbl.config(text=f"Duration:  {td}")
            self._frames_lbl.config(text=f"    Frames: {total}")
            for cam in CAMERAS:
                n = counts.get(cam, 0)
                self._cam_frame_lbls[cam].config(text=f"    {cam}: {n}")

        # Update previews for all cameras
        for cam in CAMERAS:
            frame = frames.get(cam)
            lbl = self._preview_labels[cam]
            if frame is not None:
                small = cv2.resize(frame, (PREVIEW_W, PREVIEW_H), interpolation=cv2.INTER_AREA)
                # Red border overlay while recording this camera
                if self._recording and cam in self._writers:
                    cv2.rectangle(small, (0, 0), (PREVIEW_W - 1, PREVIEW_H - 1), (0, 0, 220), 3)
                    cv2.putText(small, "REC", (8, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 220), 2, cv2.LINE_AA)
                rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
                photo = ImageTk.PhotoImage(PILImage.fromarray(rgb))
                lbl.configure(image=photo, text="", width=PREVIEW_W, height=PREVIEW_H)
                lbl.image = photo  # keep reference
            else:
                lbl.configure(image="", text=f"Waiting for\n{cam} camera…",
                               width=PREVIEW_W // 8, height=PREVIEW_H // 16)
                lbl.image = None

        self.root.after(TICK_MS, self._tick)

    # ── Recordings list ──────────────────────────────────────────────────────

    def _refresh_recordings(self):
        self._rec_list.delete(0, tk.END)
        files = sorted(VIDEOS_DIR.rglob("*.mp4"), key=lambda p: p.stat().st_mtime, reverse=True)
        for f in files:
            rel = str(f.relative_to(VIDEOS_DIR))
            size_mb = f.stat().st_size / 1_048_576
            mtime = datetime.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")
            self._rec_list.insert(tk.END, f"  {rel:<50}  {size_mb:5.1f} MB  {mtime}")

    def _open_folder(self):
        subprocess.Popen(["xdg-open", str(VIDEOS_DIR)])

    def _delete_selected(self):
        sel = self._rec_list.curselection()
        if not sel:
            messagebox.showinfo("Nothing selected", "Select a recording to delete.")
            return
        line = self._rec_list.get(sel[0]).strip()
        rel = line.split()[0]
        path = VIDEOS_DIR / rel
        if not path.exists():
            messagebox.showerror("Not found", f"{rel} not found.")
            self._refresh_recordings()
            return
        if messagebox.askyesno("Delete", f"Delete {rel}?"):
            path.unlink()
            parent = path.parent
            if parent != VIDEOS_DIR and not any(parent.iterdir()):
                parent.rmdir()
            self.node.get_logger().info(f"Deleted: {rel}")
            self._refresh_recordings()

    # ── Shutdown ─────────────────────────────────────────────────────────────

    def _on_close(self):
        if self._recording:
            if not messagebox.askyesno("Recording in progress",
                                       "A recording is active. Stop and exit?"):
                return
            self._stop_recording()
        self.root.destroy()


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = CameraRecorderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        RecorderGUI(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
