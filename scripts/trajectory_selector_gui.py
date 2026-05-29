#!/usr/bin/env python3
"""
Freespace planner trajectory selector GUI.

Mocks the teleoperation selection panel:
  - Subscribes to CandidateTrajectories and shows one button per candidate.
  - Clicking a button sends Int8(index) to the selection topic.
  - "Release" sends Int8(-1) to return to sampling mode.

Usage:
  python3 trajectory_selector_gui.py [--ros-args ...]

Topic remapping (defaults):
  candidates : /external/remote/freespace_planner/output/candidate_trajectories
  selection  : /external/remote/freespace_planner/input/selected_trajectory_index
"""

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from autoware_internal_planning_msgs.msg import CandidateTrajectories
from std_msgs.msg import Int8


CANDIDATES_TOPIC = "/external/remote/freespace_planner/output/candidate_trajectories"
SELECTION_TOPIC = "/external/remote/freespace_planner/input/selected_trajectory_index"

COLOR_DEFAULT = "#e8e8e8"
COLOR_LOCKED = "#4caf50"      # green
COLOR_FAILED = "#bdbdbd"      # grey
COLOR_HOVER = "#bbdefb"       # light blue
COLOR_RELEASE = "#ef5350"     # red
COLOR_BG = "#263238"          # dark background
COLOR_TEXT_LIGHT = "#eceff1"
COLOR_TEXT_DARK = "#212121"


class TrajectorySelectorNode(Node):
    def __init__(self, gui: "TrajectorySelectorGUI"):
        super().__init__("trajectory_selector_gui")
        self._gui = gui

        self._sub = self.create_subscription(
            CandidateTrajectories,
            CANDIDATES_TOPIC,
            self._on_candidates,
            10,
        )
        self._pub = self.create_publisher(Int8, SELECTION_TOPIC, 10)
        self.get_logger().info(
            f"Subscribing  : {CANDIDATES_TOPIC}\n"
            f"Publishing to: {SELECTION_TOPIC}"
        )

    def _on_candidates(self, msg: CandidateTrajectories):
        # Parse candidate info from generator_name:
        #   "freespace_planner/goal_Xm_path_Ym"
        entries = []
        for info, ct in zip(msg.generator_info, msg.candidate_trajectories):
            name: str = info.generator_name.data
            goal_m, path_m = self._parse_name(name)
            entries.append({
                "name": name,
                "goal_m": goal_m,
                "path_m": path_m,
                "n_pts": len(ct.points),
            })
        self._gui.update_candidates(entries)

    @staticmethod
    def _parse_name(name: str):
        """Extract goal_m and path_m from 'freespace_planner/goal_Xm_path_Ym'."""
        try:
            parts = name.split("/")[-1]  # "goal_Xm_path_Ym"
            tokens = parts.split("_")
            goal_m = float(tokens[1].replace("m", ""))
            path_m = float(tokens[3].replace("m", ""))
            return goal_m, path_m
        except Exception:
            return 0.0, 0.0

    def publish_selection(self, index: int):
        msg = Int8()
        msg.data = index
        self._pub.publish(msg)
        self.get_logger().info(f"Published selection index: {index}")


class TrajectorySelectorGUI:
    def __init__(self):
        self._root = tk.Tk()
        self._root.title("Freespace Planner — Trajectory Selector")
        self._root.configure(bg=COLOR_BG)
        self._root.resizable(False, False)

        self._node: TrajectorySelectorNode | None = None
        self._locked_idx: int = -1
        self._candidates: list[dict] = []
        self._lock = threading.Lock()

        self._build_ui()

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        # Title bar
        title_frame = tk.Frame(self._root, bg=COLOR_BG, pady=6)
        title_frame.pack(fill=tk.X, padx=12)

        bold_font = tkfont.Font(family="Helvetica", size=13, weight="bold")
        small_font = tkfont.Font(family="Helvetica", size=10)

        tk.Label(
            title_frame,
            text="Freespace Planner",
            font=bold_font,
            bg=COLOR_BG,
            fg=COLOR_TEXT_LIGHT,
        ).pack(side=tk.LEFT)

        self._status_var = tk.StringVar(value="● SAMPLING  (waiting for candidates)")
        self._status_label = tk.Label(
            title_frame,
            textvariable=self._status_var,
            font=small_font,
            bg=COLOR_BG,
            fg="#80cbc4",
        )
        self._status_label.pack(side=tk.RIGHT)

        # Separator
        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # Candidate buttons frame
        self._btn_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=8)
        self._btn_frame.pack(fill=tk.BOTH, expand=True)

        self._no_cand_label = tk.Label(
            self._btn_frame,
            text="No candidates yet — waiting for planner...",
            font=small_font,
            bg=COLOR_BG,
            fg=COLOR_FAILED,
        )
        self._no_cand_label.pack(pady=10)

        # Separator
        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # Release button
        rel_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=8)
        rel_frame.pack(fill=tk.X)

        release_font = tkfont.Font(family="Helvetica", size=10, weight="bold")
        self._release_btn = tk.Button(
            rel_frame,
            text="⟳  Release / Re-select  (send −1)",
            font=release_font,
            bg=COLOR_RELEASE,
            fg="white",
            activebackground="#c62828",
            activeforeground="white",
            relief=tk.FLAT,
            padx=10,
            pady=6,
            command=self._on_release,
        )
        self._release_btn.pack(fill=tk.X)

        self._root.after(100, self._poll_ros)

    # ── Candidate update (called from ROS thread via after()) ──────────────

    def update_candidates(self, entries: list[dict]):
        with self._lock:
            self._candidates = entries
        self._root.after(0, self._rebuild_buttons)

    def _rebuild_buttons(self):
        with self._lock:
            entries = list(self._candidates)
            locked_idx = self._locked_idx

        for widget in self._btn_frame.winfo_children():
            widget.destroy()

        btn_font = tkfont.Font(family="Helvetica", size=11)

        if not entries:
            tk.Label(
                self._btn_frame,
                text="No candidates — A* found no reachable goals.",
                font=btn_font,
                bg=COLOR_BG,
                fg=COLOR_FAILED,
            ).pack(pady=8)
            self._update_status(locked_idx)
            return

        for i, entry in enumerate(entries):
            goal_m = entry["goal_m"]
            path_m = entry["path_m"]
            is_locked = (i == locked_idx)

            label = f"  Goal {goal_m:.0f} m  —  path {path_m:.1f} m  ({entry['n_pts']} pts)  "
            bg = COLOR_LOCKED if is_locked else COLOR_DEFAULT
            fg = COLOR_TEXT_LIGHT if is_locked else COLOR_TEXT_DARK

            btn = tk.Button(
                self._btn_frame,
                text=("✔ " if is_locked else "   ") + label,
                font=btn_font,
                bg=bg,
                fg=fg,
                activebackground=COLOR_HOVER,
                activeforeground=COLOR_TEXT_DARK,
                relief=tk.FLAT,
                padx=6,
                pady=5,
                anchor=tk.W,
                command=lambda idx=i: self._on_select(idx),
            )
            btn.pack(fill=tk.X, pady=2)

        self._update_status(locked_idx)

    def _update_status(self, locked_idx: int):
        with self._lock:
            entries = self._candidates

        if locked_idx < 0 or not entries:
            self._status_var.set("● SAMPLING  (waiting for operator selection)")
            self._status_label.configure(fg="#80cbc4")
        else:
            try:
                goal_m = entries[locked_idx]["goal_m"]
                self._status_var.set(f"● LOCKED: {goal_m:.0f} m")
                self._status_label.configure(fg=COLOR_LOCKED)
            except IndexError:
                self._status_var.set("● SAMPLING  (locked index out of range)")
                self._status_label.configure(fg="#80cbc4")

    # ── Button callbacks ───────────────────────────────────────────────────

    def _on_select(self, idx: int):
        with self._lock:
            self._locked_idx = idx
        if self._node:
            self._node.publish_selection(idx)
        self._rebuild_buttons()

    def _on_release(self):
        with self._lock:
            self._locked_idx = -1
        if self._node:
            self._node.publish_selection(-1)
        self._rebuild_buttons()

    # ── ROS spin integration ───────────────────────────────────────────────

    def _poll_ros(self):
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        self._root.after(50, self._poll_ros)  # 20 Hz GUI poll

    def run(self):
        rclpy.init()
        self._node = TrajectorySelectorNode(self)
        try:
            self._root.mainloop()
        finally:
            self._node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    TrajectorySelectorGUI().run()
