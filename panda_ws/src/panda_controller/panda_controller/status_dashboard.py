#!/usr/bin/env python3
"""Live Tkinter dashboard for the shared-control simulation.

Shows, in one window:
  - joystick connection status, active coordinate frame, and teleop mode,
  - which buttons / sticks / triggers are active (with per-stick axis meaning),
  - the LiDAR tool-to-tissue distance,
  - the user-commanded vs. safety-scaled velocity side by side, so you can watch
    the safety layer slow the tool down as it approaches tissue,
  - a control guide and a button to reconnect the joystick.

Runs a ROS 2 node (spun in a background thread) and a Tk main loop that redraws
from the latest messages. It is display-only for the robot; the reconnect button
calls the /joystick/reconnect service on joy_manager.

  ros2 run panda_controller status_dashboard.py
"""

import math
import threading
import time
import tkinter as tk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


# --- colours ---
BG = '#1e1f26'
PANEL = '#2a2c37'
FG = '#e6e6e6'
MUTED = '#8a8d9a'
GREEN = '#4caf50'
RED = '#e05252'
AMBER = '#e0a03a'
ACCENT = '#4a9de0'
FONT = 'DejaVu Sans'

# Xbox-style mapping (matches tele_controller_input.cpp).
AXIS_LEFT_X, AXIS_LEFT_Y = 0, 1
AXIS_LT = 2
AXIS_RIGHT_X, AXIS_RIGHT_Y = 3, 4
AXIS_RT = 5
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_LB, BTN_RB = 4, 5

STALE_TIMEOUT = 1.0  # s without /joy before we call the pad disconnected

# Control guide shown at the bottom (label, description).
GUIDE = [
    ('RT (hold)', 'Enable motion (dead-man)'),
    ('LT (hold)', 'Precision / slow mode'),
    ('RB (hold)', 'Orientation mode (roll/pitch/yaw)'),
    ('Left stick', 'Translate X / Y  (or roll / pitch)'),
    ('Right stick', 'Translate Z / yaw  (or yaw)'),
    ('A', 'Emergency stop'),
    ('B', 'Safe joint pose'),
    ('X', 'Toggle position / velocity controller'),
    ('Y', 'Toggle frame (global / local)'),
    ('LB', 'Eye-follow: scan then follow the eye surface'),
]


class DashboardNode(Node):
    """Subscribes to the telemetry topics and caches the latest values."""

    def __init__(self):
        super().__init__('status_dashboard')
        self.latest_joy = None
        self.joy_stamp = 0.0
        self.lidar_range = None
        self.user_speed = 0.0
        self.safe_speed = 0.0
        self.frame = 'GLOBAL'
        self.eye_follow = 'DISABLED'
        self.reconnect_status = ''

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(Joy, '/joy', self._on_joy, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/probe/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/panda_velocity_cmd_user', self._on_user_cmd, 10)
        self.create_subscription(Twist, '/panda_velocity_cmd', self._on_safe_cmd, 10)
        self.create_subscription(String, '/teleop_frame', self._on_frame, latched)
        self.create_subscription(String, '/eye_follow/state', self._on_eye_follow, latched)

        self.reconnect_client = self.create_client(Trigger, '/joystick/reconnect')

    def _on_joy(self, msg):
        self.latest_joy = msg
        self.joy_stamp = time.monotonic()

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges
                 if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.lidar_range = min(valid) if valid else None

    def _on_user_cmd(self, msg):
        self.user_speed = _linear_speed(msg)

    def _on_safe_cmd(self, msg):
        self.safe_speed = _linear_speed(msg)

    def _on_frame(self, msg):
        self.frame = msg.data

    def _on_eye_follow(self, msg):
        self.eye_follow = msg.data

    def joy_connected(self):
        return (time.monotonic() - self.joy_stamp) < STALE_TIMEOUT and self.latest_joy is not None

    def request_reconnect(self):
        if not self.reconnect_client.service_is_ready():
            self.reconnect_status = 'reconnect service unavailable (joy_manager not running)'
            return
        self.reconnect_status = 'reconnecting…'
        future = self.reconnect_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_reconnect_done)

    def _on_reconnect_done(self, future):
        try:
            self.reconnect_status = future.result().message
        except Exception as exc:  # noqa: BLE001 - surface any failure in the UI
            self.reconnect_status = 'reconnect failed: %s' % exc


def _linear_speed(twist):
    return math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2)


class Dashboard:
    """The Tk window; polls the node's cached values and redraws."""

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Shared-Control Monitor')
        self.root.configure(bg=BG)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._closing = False

        self.status = tk.Label(self.root, font=(FONT, 12, 'bold'), bg=BG, fg=FG, pady=8)
        self.status.pack(fill='x')

        self.pad = tk.Canvas(self.root, width=460, height=250, bg=PANEL, highlightthickness=0)
        self.pad.pack(padx=10, pady=(0, 8))

        self.bars = tk.Canvas(self.root, width=460, height=170, bg=PANEL, highlightthickness=0)
        self.bars.pack(padx=10, pady=(0, 8))

        self._build_guide()
        self._tick()

    # --- guide / reconnect ----------------------------------------------
    def _build_guide(self):
        container = tk.Frame(self.root, bg=PANEL, padx=12, pady=8)
        container.pack(padx=10, pady=(0, 10), fill='x')

        # Control bar: an info toggle on the left, reconnect on the right.
        bar = tk.Frame(container, bg=PANEL)
        bar.pack(fill='x')

        self.guide_visible = False
        self.toggle_btn = tk.Button(
            bar, text='ⓘ  Controls  ▾', command=self._toggle_guide,
            bg=PANEL, fg=ACCENT, activebackground=PANEL, activeforeground=FG,
            relief='flat', bd=0, cursor='hand2', font=(FONT, 10, 'bold'))
        self.toggle_btn.pack(side='left')

        tk.Button(
            bar, text='⟳  Reconnect joystick', command=self._on_reconnect_click,
            bg=ACCENT, fg='white', activebackground=ACCENT, activeforeground='white',
            relief='flat', cursor='hand2', padx=12, pady=3,
            font=(FONT, 10, 'bold')).pack(side='right')

        # Reconnect status pinned to the bottom.
        self.reconnect_label = tk.Label(container, text='', font=(FONT, 9),
                                        bg=PANEL, fg=MUTED, anchor='w')
        self.reconnect_label.pack(side='bottom', fill='x', pady=(6, 0))

        # Collapsible control guide (hidden until the info toggle is pressed).
        self.guide_frame = tk.Frame(container, bg=PANEL)
        for i, (key, desc) in enumerate(GUIDE):
            tk.Label(self.guide_frame, text=key, font=(FONT, 9, 'bold'), bg=PANEL,
                     fg=ACCENT, anchor='w', width=12).grid(row=i, column=0, sticky='w', pady=1)
            tk.Label(self.guide_frame, text=desc, font=(FONT, 9), bg=PANEL,
                     fg=FG, anchor='w').grid(row=i, column=1, sticky='w', pady=1)

    def _toggle_guide(self):
        self.guide_visible = not self.guide_visible
        if self.guide_visible:
            self.guide_frame.pack(fill='x', pady=(8, 0))
            self.toggle_btn.config(text='ⓘ  Controls  ▴')
        else:
            self.guide_frame.pack_forget()
            self.toggle_btn.config(text='ⓘ  Controls  ▾')

    def _on_reconnect_click(self):
        self.node.request_reconnect()

    # --- input helpers ---------------------------------------------------
    def _axis(self, idx, default=0.0):
        joy = self.node.latest_joy
        if joy and idx < len(joy.axes):
            return joy.axes[idx]
        return default

    def _button(self, idx):
        joy = self.node.latest_joy
        return bool(joy and idx < len(joy.buttons) and joy.buttons[idx])

    def _trigger(self, idx):
        # Triggers rest at +1 and go to -1 when pressed.
        return (1.0 - self._axis(idx, 1.0)) / 2.0

    # --- drawing ---------------------------------------------------------
    def _draw_status(self, connected):
        emergency = connected and self._button(BTN_A)
        orient = connected and self._button(BTN_RB)
        conn_txt = '● CONNECTED' if connected else '○ DISCONNECTED'
        mode = 'ORIENT' if orient else 'TRANSLATE'
        text = 'Joystick: %s     Mode: %s     Frame: %s' % (conn_txt, mode, self.node.frame)
        if self.node.eye_follow and self.node.eye_follow != 'DISABLED':
            text += '     Eye-follow: %s' % self.node.eye_follow
        if emergency:
            text += '     EMERGENCY STOP'
        self.status.config(text=text, fg=RED if emergency else (GREEN if connected else RED))

    def _draw_button(self, cx, cy, r, pressed, label, on_col=GREEN):
        fill = on_col if pressed else '#3a3d4a'
        self.pad.create_oval(cx - r, cy - r, cx + r, cy + r, fill=fill, outline=MUTED)
        self.pad.create_text(cx, cy, text=label, fill=FG, font=(FONT, 10, 'bold'))

    def _draw_stick(self, cx, cy, r, ax, ay, axis_label):
        self.pad.create_oval(cx - r, cy - r, cx + r, cy + r, outline=MUTED)
        px = cx + ax * r
        py = cy - ay * r  # up = +1 for display
        self.pad.create_line(cx, cy, px, py, fill=ACCENT)
        self.pad.create_oval(px - 6, py - 6, px + 6, py + 6, fill=ACCENT, outline='')
        self.pad.create_text(cx, cy + r + 16, text=axis_label, fill=ACCENT, font=(FONT, 9, 'bold'))

    def _draw_trigger(self, cx, top, w, h, value, label):
        x = cx - w / 2
        self.pad.create_text(cx, top - 11, text=label, fill=FG, font=(FONT, 9, 'bold'))
        self.pad.create_rectangle(x, top, x + w, top + h, outline=MUTED)
        fh = h * max(0.0, min(1.0, value))
        self.pad.create_rectangle(x, top + h - fh, x + w, top + h, fill=AMBER, outline='')
        self.pad.create_text(cx, top + h + 12, text='%.2f' % value, fill=MUTED, font=(FONT, 9))

    def _draw_pad(self, connected):
        self.pad.delete('all')
        if not connected:
            self.pad.create_text(230, 125, text='waiting for /joy…',
                                 fill=MUTED, font=(FONT, 13))
            return
        orient = self._button(BTN_RB)
        # Triggers (left edge, clearly separated labels).
        self._draw_trigger(35, 55, 26, 110, self._trigger(AXIS_LT), 'LT')
        self._draw_trigger(92, 55, 26, 110, self._trigger(AXIS_RT), 'RT')
        # Bumpers.
        self._draw_button(165, 35, 16, self._button(BTN_LB), 'LB')
        self._draw_button(300, 35, 16, self._button(BTN_RB), 'RB', on_col=ACCENT)
        # Face buttons (diamond, right side).
        self._draw_button(410, 90, 18, self._button(BTN_Y), 'Y', on_col=AMBER)
        self._draw_button(380, 120, 18, self._button(BTN_X), 'X', on_col=ACCENT)
        self._draw_button(440, 120, 18, self._button(BTN_B), 'B', on_col=RED)
        self._draw_button(410, 150, 18, self._button(BTN_A), 'A', on_col=GREEN)
        # Sticks with mode-aware axis meaning.
        left_axes = 'Roll / Pitch' if orient else 'X / Y'
        right_axes = 'Yaw' if orient else 'Z / Yaw'
        self._draw_stick(175, 130, 45,
                         self._axis(AXIS_LEFT_X), self._axis(AXIS_LEFT_Y), left_axes)
        self._draw_stick(285, 130, 45,
                         self._axis(AXIS_RIGHT_X), self._axis(AXIS_RIGHT_Y), right_axes)

    def _draw_bar(self, y, label, value, vmax, text, colour):
        c = self.bars
        x0, x1, h = 130, 440, 20
        c.create_text(15, y + h / 2, text=label, fill=FG, anchor='w', font=(FONT, 10))
        c.create_rectangle(x0, y, x1, y + h, outline=MUTED)
        frac = 0.0 if vmax <= 0 else max(0.0, min(1.0, value / vmax))
        c.create_rectangle(x0, y, x0 + (x1 - x0) * frac, y + h, fill=colour, outline='')
        c.create_text((x0 + x1) / 2, y + h / 2, text=text, fill=FG, font=(FONT, 9, 'bold'))

    def _draw_bars(self):
        c = self.bars
        c.delete('all')
        rng = self.node.lidar_range
        if rng is None:
            self._draw_bar(20, 'LiDAR', 0.0, 0.1, 'no reading', '#3a3d4a')
        else:
            colour = RED if rng < 0.02 else (AMBER if rng < 0.05 else GREEN)
            # Invert so a full bar means close (more danger).
            self._draw_bar(20, 'LiDAR', max(0.0, 0.1 - rng), 0.1, '%.3f m' % rng, colour)
        user = self.node.user_speed
        safe = self.node.safe_speed
        self._draw_bar(70, 'User |v|', user, 0.3, '%.3f m/s' % user, ACCENT)
        scale_txt = '%.3f m/s' % safe
        if user > 1e-4:
            scale_txt += '   (%.0f%% of user)' % (100.0 * safe / user)
        self._draw_bar(120, 'Safe |v|', safe, 0.3, scale_txt,
                       GREEN if safe >= user - 1e-4 else AMBER)

    def _tick(self):
        if self._closing:
            return
        connected = self.node.joy_connected()
        self._draw_status(connected)
        self._draw_pad(connected)
        self._draw_bars()
        self.reconnect_label.config(text=self.node.reconnect_status)
        self.root.after(50, self._tick)  # ~20 Hz

    def _on_close(self):
        self._closing = True
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = DashboardNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:
            pass  # executor.shutdown() unblocks spin() on exit

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    try:
        Dashboard(node).run()
    except tk.TclError as exc:
        node.get_logger().error(
            'Could not open the dashboard window (no display?): %s' % exc)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
