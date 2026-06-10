#!/usr/bin/env python3
"""
Freespace planner trajectory selector GUI.

  - Subscribes to CandidateTrajectories and shows one button per candidate.
  - Clicking a button publishes Int32(index) to the desired_trajectory_index topic.

Topic defaults (resolved under /external/remote/freespace_planner/):
  candidates : ~/output/candidate_trajectories
  selection  : ~/input/desired_trajectory_index
  state      : ~/output/current_state
"""

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_adapi_v1_msgs.srv import MuxSelectInput
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


CANDIDATES_TOPIC = "/external/remote/freespace_planner/output/candidate_trajectories"
SELECTION_TOPIC  = "/external/remote/freespace_planner/input/desired_trajectory_index"
TRIGGER_SERVICE  = "/external/remote/freespace_planner/trigger_replan"
STATE_TOPIC      = "/external/remote/freespace_planner/output/current_state"
MUX_SERVICE      = "/planning/trajectory_multiplexer/select_input"

COLOR_DEFAULT    = "#e8e8e8"
COLOR_LOCKED     = "#4caf50"
COLOR_FAILED     = "#bdbdbd"
COLOR_HOVER      = "#bbdefb"
COLOR_BG         = "#263238"
COLOR_TEXT_LIGHT = "#eceff1"
COLOR_TEXT_DARK  = "#212121"
COLOR_PLAN_BTN   = "#1565c0"
COLOR_PLAN_PEND  = "#ffb300"
COLOR_PLAN_OK    = "#2e7d32"
COLOR_PLAN_FAIL  = "#b71c1c"


class TrajectorySelectorNode(Node):
    def __init__(self, gui: "TrajectorySelectorGUI"):
        super().__init__("trajectory_selector_gui")
        self._gui = gui

        self._sub = self.create_subscription(
            CandidateTrajectories, CANDIDATES_TOPIC, self._on_candidates, 10)
        self._pub = self.create_publisher(Int32, SELECTION_TOPIC, 10)
        self._cli = self.create_client(Trigger, TRIGGER_SERVICE)
        self._mux_cli = self.create_client(MuxSelectInput, MUX_SERVICE)
        self._state_sub = self.create_subscription(
            String, STATE_TOPIC, self._on_state, 10)

        self.get_logger().info(
            f"Subscribing  : {CANDIDATES_TOPIC}\n"
            f"             : {STATE_TOPIC}\n"
            f"Publishing to: {SELECTION_TOPIC}\n"
            f"Service      : {TRIGGER_SERVICE}\n"
            f"             : {MUX_SERVICE}"
        )

    def _on_candidates(self, msg: CandidateTrajectories):
        self._gui.update_candidates(len(msg.candidate_trajectories))

    def _on_state(self, msg: String):
        self._gui.update_state(msg.data)

    def publish_selection(self, index: int):
        msg = Int32()
        msg.data = index
        self._pub.publish(msg)
        self.get_logger().info(f"Published desired_trajectory_index: {index}")

    def trigger_replan(self, done_cb):
        if not self._cli.service_is_ready():
            self.get_logger().warn("trigger_replan service not available")
            done_cb(False, "Service not available")
            return
        future = self._cli.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_trigger_response(f, done_cb))

    def _on_trigger_response(self, future, done_cb):
        try:
            result = future.result()
            self.get_logger().info(f"trigger_replan: {result.message}")
            done_cb(result.success, result.message)
        except Exception as e:
            self.get_logger().error(f"trigger_replan failed: {e}")
            done_cb(False, str(e))

    def select_mux_input(self, mode: str, done_cb):
        if not self._mux_cli.service_is_ready():
            self.get_logger().warn(f"mux service not available")
            done_cb(False)
            return
        req = MuxSelectInput.Request()
        req.select_input = mode
        future = self._mux_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_mux_response(f, done_cb))

    def _on_mux_response(self, future, done_cb):
        try:
            result = future.result()
            self.get_logger().info(f"mux select: success={result.success}")
            done_cb(result.success)
        except Exception as e:
            self.get_logger().error(f"mux select failed: {e}")
            done_cb(False)


class TrajectorySelectorGUI:
    def __init__(self):
        self._root = tk.Tk()
        self._root.title("Freespace Planner — Trajectory Selector")
        self._root.configure(bg=COLOR_BG)
        self._root.resizable(False, False)

        self._node: TrajectorySelectorNode | None = None
        self._locked_idx: int = -1
        self._n_candidates: int = 0
        self._buttons: list[tk.Button] = []
        self._lock = threading.Lock()

        self._build_ui()

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        small_font      = tkfont.Font(family="Helvetica", size=10)
        state_val_font  = tkfont.Font(family="Helvetica", size=10, weight="bold")
        plan_font       = tkfont.Font(family="Helvetica", size=12, weight="bold")

        # State row
        state_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=8)
        state_frame.pack(fill=tk.X)
        tk.Label(state_frame, text="State:", font=small_font,
                 bg=COLOR_BG, fg="#78909c").pack(side=tk.LEFT)
        self._state_var = tk.StringVar(value="—")
        tk.Label(state_frame, textvariable=self._state_var,
                 font=state_val_font,
                 bg=COLOR_BG, fg=COLOR_TEXT_LIGHT).pack(side=tk.LEFT, padx=(6, 0))

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # Planning request button
        plan_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=10)
        plan_frame.pack(fill=tk.X)
        self._plan_btn = tk.Button(
            plan_frame,
            text="▶  Send Planning Request",
            font=plan_font,
            bg=COLOR_PLAN_BTN, fg="white",
            activebackground="#1976d2", activeforeground="white",
            relief=tk.FLAT, padx=10, pady=8,
            command=self._on_trigger_replan,
        )
        self._plan_btn.pack(fill=tk.X)

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # Candidate buttons frame
        self._btn_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=8)
        self._btn_frame.pack(fill=tk.BOTH, expand=True)
        tk.Label(self._btn_frame,
                 text="No candidates yet — waiting for planner...",
                 font=small_font, bg=COLOR_BG, fg=COLOR_FAILED).pack(pady=10)

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # Publish selection button
        pub_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=10)
        pub_frame.pack(fill=tk.X)
        self._publish_btn = tk.Button(
            pub_frame,
            text="Publish Selection",
            font=plan_font,
            bg=COLOR_PLAN_OK, fg="white",
            activebackground="#1b5e20", activeforeground="white",
            disabledforeground="#78909c",
            relief=tk.FLAT, padx=10, pady=8,
            state=tk.DISABLED,
            command=self._on_publish_selection,
        )
        self._publish_btn.pack(fill=tk.X)

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # Mux select buttons
        mux_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=10)
        mux_frame.pack(fill=tk.X)
        mux_font = tkfont.Font(family="Helvetica", size=11, weight="bold")

        self._remote_btn = tk.Button(
            mux_frame,
            text="Set Remote",
            font=mux_font,
            bg="#e65100", fg="white",
            activebackground="#bf360c", activeforeground="white",
            disabledforeground="#78909c",
            relief=tk.FLAT, padx=8, pady=7,
            command=lambda: self._on_mux_select("REMOTE", self._remote_btn, "#e65100", "Set Remote"),
        )
        self._remote_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

        self._auto_btn = tk.Button(
            mux_frame,
            text="Set Auto",
            font=mux_font,
            bg="#00695c", fg="white",
            activebackground="#004d40", activeforeground="white",
            disabledforeground="#78909c",
            relief=tk.FLAT, padx=8, pady=7,
            command=lambda: self._on_mux_select("AUTO", self._auto_btn, "#00695c", "Set Auto"),
        )
        self._auto_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4, 0))

        # Fix window to the width required by the longest state string.
        # state_frame has padx=12 on each side; the value label has padx=(6,0).
        _fixed_w = (12 + small_font.measure("State:") + 6
                    + state_val_font.measure("AWAITING_TRAJECTORY_SELECTION") + 12)
        self._root.minsize(_fixed_w, 0)
        self._root.maxsize(_fixed_w, 9999)

        self._root.after(100, self._poll_ros)

    # ── Candidate / state updates ──────────────────────────────────────────

    def update_state(self, state: str):
        self._root.after(0, lambda: self._state_var.set(state))

    def update_candidates(self, count: int):
        with self._lock:
            if count == self._n_candidates:
                return
            self._n_candidates = count
            if self._locked_idx >= count:
                self._locked_idx = -1
        self._root.after(0, self._rebuild_buttons)

    def _rebuild_buttons(self):
        with self._lock:
            n = self._n_candidates
            locked_idx = self._locked_idx

        for w in self._btn_frame.winfo_children():
            w.destroy()
        self._buttons.clear()
        self._publish_btn.configure(state=tk.DISABLED)

        btn_font = tkfont.Font(family="Helvetica", size=11)

        if n == 0:
            tk.Label(self._btn_frame,
                     text="No candidates — A* found no reachable goals.",
                     font=btn_font, bg=COLOR_BG, fg=COLOR_FAILED).pack(pady=8)
            return

        for i in range(n):
            is_locked = (i == locked_idx)
            btn = tk.Button(
                self._btn_frame,
                text=("✔ " if is_locked else "    ") + f"Trajectory {i}",
                font=btn_font,
                bg=COLOR_LOCKED if is_locked else COLOR_DEFAULT,
                fg=COLOR_TEXT_LIGHT if is_locked else COLOR_TEXT_DARK,
                activebackground=COLOR_HOVER,
                activeforeground=COLOR_TEXT_DARK,
                relief=tk.FLAT, padx=6, pady=5, anchor=tk.W,
                command=lambda idx=i: self._on_select(idx),
            )
            btn.pack(fill=tk.X, pady=2)
            self._buttons.append(btn)

    def _update_button_styles(self):
        with self._lock:
            locked_idx = self._locked_idx
        for i, btn in enumerate(self._buttons):
            is_locked = (i == locked_idx)
            btn.configure(
                text=("✔ " if is_locked else "    ") + f"Trajectory {i}",
                bg=COLOR_LOCKED if is_locked else COLOR_DEFAULT,
                fg=COLOR_TEXT_LIGHT if is_locked else COLOR_TEXT_DARK,
            )
        self._publish_btn.configure(
            state=tk.NORMAL if locked_idx >= 0 else tk.DISABLED)

    # ── Button callbacks ───────────────────────────────────────────────────

    def _on_trigger_replan(self):
        self._plan_btn.configure(text="⟳  Sending...", bg=COLOR_PLAN_PEND, state=tk.DISABLED)
        if self._node:
            self._node.trigger_replan(self._on_trigger_done)
        else:
            self._root.after(1500, self._reset_plan_btn)

    def _on_trigger_done(self, success: bool, message: str):
        color = COLOR_PLAN_OK if success else COLOR_PLAN_FAIL
        text  = "✔  Request Sent" if success else "✗  Failed"
        self._root.after(0, lambda: self._plan_btn.configure(
            text=text, bg=color, state=tk.NORMAL))
        self._root.after(2000, self._reset_plan_btn)

    def _reset_plan_btn(self):
        self._plan_btn.configure(
            text="▶  Send Planning Request", bg=COLOR_PLAN_BTN, state=tk.NORMAL)

    def _on_select(self, idx: int):
        with self._lock:
            self._locked_idx = idx
        self._update_button_styles()

    def _on_publish_selection(self):
        with self._lock:
            idx = self._locked_idx
        if idx < 0:
            return
        if self._node:
            self._node.publish_selection(idx)
        with self._lock:
            self._locked_idx = -1
        self._update_button_styles()

    def _on_mux_select(self, mode: str, btn: tk.Button, orig_bg: str, orig_text: str):
        btn.configure(text="⟳  ...", bg=COLOR_PLAN_PEND, state=tk.DISABLED)
        if self._node:
            self._node.select_mux_input(
                mode,
                lambda ok: self._root.after(0, lambda: self._finish_mux(btn, ok, orig_bg, orig_text)),
            )
        else:
            self._root.after(1500, lambda: self._finish_mux(btn, False, orig_bg, orig_text))

    def _finish_mux(self, btn: tk.Button, success: bool, orig_bg: str, orig_text: str):
        btn.configure(
            text="✔" if success else "✗",
            bg=COLOR_PLAN_OK if success else COLOR_PLAN_FAIL,
            state=tk.NORMAL,
        )
        self._root.after(1500, lambda: btn.configure(text=orig_text, bg=orig_bg))

    # ── ROS spin integration ───────────────────────────────────────────────

    def _poll_ros(self):
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        self._root.after(50, self._poll_ros)  # 20 Hz

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
