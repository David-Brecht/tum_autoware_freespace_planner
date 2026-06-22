#!/usr/bin/env python3
"""
Freespace planner trajectory selector GUI.

  - Subscribes to CandidateTrajectories and shows one button per candidate.
  - Clicking a button then "Publish Selection" sends Int32(index) to the planner.

Topic / service defaults (resolved under /external/remote/freespace_planner/):
  candidates       : ~/output/candidate_trajectories
  selection        : ~/input/desired_trajectory_index
  planner state    : ~/output/current_state
  autoware state   : /api/motion/state
"""

import signal
import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_adapi_v1_msgs.msg import MotionState, TeleoperationState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, MuxSelectInput
from autoware_vehicle_msgs.msg import Engage
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


CANDIDATES_TOPIC  = "/external/remote/freespace_planner/output/candidate_trajectories"
SELECTION_TOPIC   = "/external/remote/freespace_planner/input/desired_trajectory_index"
TRIGGER_SERVICE   = "/external/remote/freespace_planner/trigger_replan"
STATE_TOPIC       = "/external/remote/freespace_planner/output/current_state"
MOTION_STATE_TOPIC        = "/api/motion/state"
TELEOPERATION_STATE_TOPIC = "/external/remote/teleoperation_state"
MUX_SERVICE               = "/planning/scenario_planning/lane_driving/behavior_planning/path_multiplexer/select_input"
ENGAGE_TOPIC              = "/autoware/engage"
CHANGE_TO_STOP_SERVICE    = "/api/operation_mode/change_to_stop"
CHANGE_TO_REMOTE_SERVICE  = "/api/operation_mode/change_to_remote"
CHANGE_TO_AUTO_SERVICE    = "/api/operation_mode/change_to_autonomous"

_MOTION_STATES = {
    MotionState.UNKNOWN:  "UNKNOWN",
    MotionState.STOPPED:  "STOPPED",
    MotionState.STARTING: "STARTING",
    MotionState.MOVING:   "MOVING",
}

COLOR_DEFAULT    = "#e8e8e8"
COLOR_LOCKED     = "#4caf50"
COLOR_FAILED     = "#bdbdbd"
COLOR_HOVER      = "#bbdefb"

# Trajectory button palette — 4 colors, cycling (index % 4)
# Normal:  source RGB blended on white at alpha=0.6  →  0.6*src + 0.4*255
# Locked:  full saturation (alpha=1.0) to clearly mark selection
_TRAJ_NORMAL = ["#FF9966", "#FFCC66", "#FFFF66", "#66FF66"]
_TRAJ_LOCKED = ["#FF5500", "#FFAA00", "#FFFF00", "#00FF00"]
COLOR_BG         = "#263238"
COLOR_TEXT_LIGHT = "#eceff1"
COLOR_TEXT_DARK  = "#212121"
COLOR_SVC_BTN    = "#00796b"   # unified color for all service-call buttons
COLOR_SVC_ACTIVE = "#004d40"
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
        self._state_sub = self.create_subscription(
            String, STATE_TOPIC, self._on_state, 10)
        self._motion_sub = self.create_subscription(
            MotionState, MOTION_STATE_TOPIC, self._on_motion_state, 10)

        self._engage_pub = self.create_publisher(Engage, ENGAGE_TOPIC, 1)
        self._teleop_pub = self.create_publisher(TeleoperationState, TELEOPERATION_STATE_TOPIC, 1)
        self.create_timer(1.0, self._publish_teleop_state)

        self._replan_cli = self.create_client(Trigger, TRIGGER_SERVICE)
        self._mux_cli = self.create_client(MuxSelectInput, MUX_SERVICE)
        self._change_to_stop_cli = self.create_client(ChangeOperationMode, CHANGE_TO_STOP_SERVICE)
        self._change_to_remote_cli = self.create_client(ChangeOperationMode, CHANGE_TO_REMOTE_SERVICE)
        self._change_to_auto_cli = self.create_client(ChangeOperationMode, CHANGE_TO_AUTO_SERVICE)

        self.get_logger().info(
            f"Subscribing  : {CANDIDATES_TOPIC}\n"
            f"             : {STATE_TOPIC}\n"
            f"             : {MOTION_STATE_TOPIC}\n"
            f"Publishing to: {SELECTION_TOPIC}\n"
            f"Services     : {TRIGGER_SERVICE}\n"
            f"             : {MUX_SERVICE}\n"
            f"             : {CHANGE_TO_STOP_SERVICE}\n"
            f"             : {CHANGE_TO_REMOTE_SERVICE}\n"
            f"             : {CHANGE_TO_AUTO_SERVICE}"
        )

    def _publish_teleop_state(self):
        msg = TeleoperationState()
        msg.state = 2
        msg.mode = 3
        self._teleop_pub.publish(msg)

    # ── Subscription callbacks ─────────────────────────────────────────────

    def _on_candidates(self, msg: CandidateTrajectories):
        self._gui.update_candidates(len(msg.candidate_trajectories))

    def _on_state(self, msg: String):
        self._gui.update_planner_state(msg.data)

    def _on_motion_state(self, msg: MotionState):
        self._gui.update_autoware_state(
            _MOTION_STATES.get(msg.state, f"UNKNOWN({msg.state})"))

    # ── Service calls ──────────────────────────────────────────────────────

    def publish_selection(self, index: int):
        msg = Int32()
        msg.data = index
        self._pub.publish(msg)
        self.get_logger().info(f"Published desired_trajectory_index: {index}")

    def trigger_replan(self, done_cb):
        if not self._replan_cli.service_is_ready():
            self.get_logger().warn("trigger_replan service not available")
            done_cb(False, "Service not available")
            return
        future = self._replan_cli.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_replan_resp(f, done_cb))

    def _on_replan_resp(self, future, done_cb):
        try:
            r = future.result()
            done_cb(r.success, r.message)
        except Exception as e:
            self.get_logger().error(f"trigger_replan failed: {e}")
            done_cb(False, str(e))

    def call_change_to_remote(self, done_cb):
        self._call_op_mode(self._change_to_remote_cli, done_cb)

    def call_change_to_stop(self, done_cb):
        self._call_op_mode(self._change_to_stop_cli, done_cb)

    def call_change_to_auto(self, done_cb):
        self._call_op_mode(self._change_to_auto_cli, done_cb)

    def call_mux_remote(self, done_cb):
        self._select_mux("REMOTE", done_cb)

    def call_mux_auto(self, done_cb):
        self._select_mux("AUTO", done_cb)

    def force_engage_and_mux_auto(self, done_cb):
        msg = Engage()
        msg.engage = True
        self._engage_pub.publish(msg)
        self.get_logger().info("Published engage=True to /autoware/engage")
        self._select_mux("AUTO", done_cb)

    def _call_op_mode(self, cli, done_cb):
        if not cli.service_is_ready():
            self.get_logger().warn("operation mode service not available")
            done_cb(False)
            return
        future = cli.call_async(ChangeOperationMode.Request())
        future.add_done_callback(lambda f: self._on_change_op_resp(f, done_cb))

    def _on_change_op_resp(self, future, done_cb):
        try:
            r = future.result()
            self.get_logger().info(f"change_operation_mode: success={r.status.success}")
            done_cb(r.status.success)
        except Exception as e:
            self.get_logger().error(f"change_operation_mode failed: {e}")
            done_cb(False)

    def _select_mux(self, mode: str, done_cb):
        if not self._mux_cli.service_is_ready():
            self.get_logger().warn("mux service not available")
            done_cb(False)
            return
        req = MuxSelectInput.Request()
        req.select_input = mode
        future = self._mux_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_mux_resp(f, done_cb))

    def _on_mux_resp(self, future, done_cb):
        try:
            r = future.result()
            self.get_logger().info(f"mux select: success={r.success}")
            done_cb(r.success)
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
        small_font     = tkfont.Font(family="Helvetica", size=10)
        state_val_font = tkfont.Font(family="Helvetica", size=10, weight="bold")
        plan_font      = tkfont.Font(family="Helvetica", size=12, weight="bold")

        # Fix window width to the rendered width of the longest planner-state string.
        _fixed_w = (12 + small_font.measure("Planner state:") + 6
                    + state_val_font.measure("AWAITING_TRAJECTORY_SELECTION") + 12)
        self._root.minsize(_fixed_w, 0)
        self._root.maxsize(_fixed_w, 9999)

        # Button wraplength: window width minus frame padx (12 each side) and button padx
        _wrap = _fixed_w - 24 - 20

        # ── State rows ──────────────────────────────────────────────────────

        state_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=6)
        state_frame.pack(fill=tk.X)

        tk.Label(state_frame, text="Planner state:", font=small_font,
                 bg=COLOR_BG, fg="#78909c").grid(row=0, column=0, sticky=tk.W)
        self._planner_state_var = tk.StringVar(value="—")
        tk.Label(state_frame, textvariable=self._planner_state_var, font=state_val_font,
                 bg=COLOR_BG, fg=COLOR_TEXT_LIGHT).grid(row=0, column=1, sticky=tk.W, padx=(6, 0))

        tk.Label(state_frame, text="Autoware state:", font=small_font,
                 bg=COLOR_BG, fg="#78909c").grid(row=1, column=0, sticky=tk.W, pady=(3, 0))
        self._autoware_state_var = tk.StringVar(value="—")
        tk.Label(state_frame, textvariable=self._autoware_state_var, font=state_val_font,
                 bg=COLOR_BG, fg=COLOR_TEXT_LIGHT).grid(row=1, column=1, sticky=tk.W,
                                                         padx=(6, 0), pady=(3, 0))

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # ── Set to Remote ────────────────────────────────────────────────────

        remote_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=10)
        remote_frame.pack(fill=tk.X)
        self._action_btn(remote_frame, "Set Autoware to Remote",
                         lambda cb: self._node.call_change_to_remote(cb),
                         plan_font, _wrap).pack(fill=tk.X, pady=(0, 4))
        self._action_btn(remote_frame, "Set Path Multiplexer to Remote",
                         lambda cb: self._node.call_mux_remote(cb),
                         plan_font, _wrap).pack(fill=tk.X)

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # ── Planning request ─────────────────────────────────────────────────

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

        # ── Candidate buttons ────────────────────────────────────────────────

        self._btn_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=8)
        self._btn_frame.pack(fill=tk.BOTH, expand=True)
        tk.Label(self._btn_frame,
                 text="No candidates yet — waiting for planner...",
                 font=small_font, bg=COLOR_BG, fg=COLOR_FAILED).pack(pady=10)

        tk.Frame(self._root, bg="#546e7a", height=1).pack(fill=tk.X, padx=12)

        # ── Publish selection ────────────────────────────────────────────────

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

        # ── Set to Auto ──────────────────────────────────────────────────────

        auto_frame = tk.Frame(self._root, bg=COLOR_BG, padx=12, pady=10)
        auto_frame.pack(fill=tk.X)
        self._action_btn(auto_frame, "Set Autoware to Stop",
                         lambda cb: self._node.call_change_to_stop(cb),
                         plan_font, _wrap).pack(fill=tk.X, pady=(0, 4))
        self._action_btn(auto_frame, "Set Autoware to Auto",
                         lambda cb: self._node.call_change_to_auto(cb),
                         plan_font, _wrap).pack(fill=tk.X, pady=(0, 4))
        self._action_btn(auto_frame, "Set Path Multiplexer to Auto",
                         lambda cb: self._node.call_mux_auto(cb),
                         plan_font, _wrap).pack(fill=tk.X, pady=(0, 4))
        self._action_btn(auto_frame, "Force Engage Auto + Set Path Multiplexer to Auto",
                         lambda cb: self._node.force_engage_and_mux_auto(cb),
                         plan_font, _wrap).pack(fill=tk.X)

        self._root.after(100, self._poll_ros)

    # ── State / candidate updates ──────────────────────────────────────────

    def update_planner_state(self, state: str):
        self._root.after(0, lambda: self._planner_state_var.set(state))

    def update_autoware_state(self, state: str):
        self._root.after(0, lambda: self._autoware_state_var.set(state))

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
            col_n = _TRAJ_NORMAL[i % len(_TRAJ_NORMAL)]
            col_l = _TRAJ_LOCKED[i % len(_TRAJ_LOCKED)]
            btn = tk.Button(
                self._btn_frame,
                text=("✔ " if is_locked else "    ") + f"Trajectory {i}",
                font=btn_font,
                bg=col_l if is_locked else col_n,
                fg=COLOR_TEXT_DARK,
                activebackground=col_l,
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
            col_n = _TRAJ_NORMAL[i % len(_TRAJ_NORMAL)]
            col_l = _TRAJ_LOCKED[i % len(_TRAJ_LOCKED)]
            btn.configure(
                text=("✔ " if is_locked else "    ") + f"Trajectory {i}",
                bg=col_l if is_locked else col_n,
                fg=COLOR_TEXT_DARK,
            )
        self._publish_btn.configure(
            state=tk.NORMAL if locked_idx >= 0 else tk.DISABLED)

    # ── Button callbacks ───────────────────────────────────────────────────

    def _on_trigger_replan(self):
        self._plan_btn.configure(text="⟳  Sending...", bg=COLOR_PLAN_PEND, state=tk.DISABLED)
        if self._node:
            self._node.trigger_replan(self._on_replan_done)
        else:
            self._root.after(1500, self._reset_plan_btn)

    def _on_replan_done(self, success: bool, *_):
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

    def _action_btn(self, parent, text: str, node_fn, plan_font, wrap: int) -> tk.Button:
        btn = tk.Button(parent, text=text, font=plan_font, wraplength=wrap,
                        bg=COLOR_SVC_BTN, fg="white",
                        activebackground=COLOR_SVC_ACTIVE, activeforeground="white",
                        relief=tk.FLAT, padx=10, pady=8)
        def _cmd():
            self._start_mode_btn(btn)
            if self._node:
                node_fn(lambda ok: self._root.after(0, lambda: self._finish_mode_btn(btn, ok, COLOR_SVC_BTN, text)))
            else:
                self._root.after(1500, lambda: self._finish_mode_btn(btn, False, COLOR_SVC_BTN, text))
        btn.configure(command=_cmd)
        return btn

    def _start_mode_btn(self, btn: tk.Button):
        btn.configure(text="⟳  ...", bg=COLOR_PLAN_PEND, state=tk.DISABLED)

    def _finish_mode_btn(self, btn: tk.Button, success: bool, orig_bg: str, orig_text: str):
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
        signal.signal(signal.SIGINT, lambda *_: self._root.after(0, self._root.quit))
        try:
            self._root.mainloop()
        finally:
            self._node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    TrajectorySelectorGUI().run()
