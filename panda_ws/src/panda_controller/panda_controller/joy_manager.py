#!/usr/bin/env python3
"""Supervisor for the joystick driver.

The stock joy_node keeps the handle it opened at startup, so if the controller
is unplugged and replugged it never re-establishes communication. This node owns
a single joy_node child process and restarts it cleanly *on demand* (the
/joystick/reconnect service, wired to the dashboard button): it stops the old
driver, waits briefly so the device handle is released, then starts a fresh one
that re-grabs whatever controller is currently connected.

It deliberately does NOT poll /joy and restart on silence — a fresh joy_node
needs to run undisturbed to open the device, and an aggressive watchdog just
races the device handle and never settles. joy_node's own output is forwarded so
connection problems are visible on the console.

  Service:  /joystick/reconnect  (std_srvs/srv/Trigger)  -> restart the driver

  ros2 run panda_controller joy_manager.py
"""

import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class JoyManager(Node):

    def __init__(self):
        super().__init__('joy_manager')

        self.joy_package = self.declare_parameter('joy_package', 'joy').value
        self.joy_executable = self.declare_parameter('joy_executable', 'joy_node').value
        self.device_id = self.declare_parameter('device_id', 0).value
        self.deadzone = self.declare_parameter('deadzone', 0.05).value
        self.autorepeat_rate = self.declare_parameter('autorepeat_rate', 20.0).value
        # Delay between stopping the old driver and starting a new one, so the
        # device file handle is released before the fresh joy_node opens it.
        self.restart_delay = self.declare_parameter('restart_delay', 1.0).value

        self._proc = None
        self._reader = None
        self._exit_reported = False

        self.reconnect_srv = self.create_service(
            Trigger, '/joystick/reconnect', self._on_reconnect)
        # Light timer only to report (once) if the driver dies; no auto-restart.
        self.create_timer(2.0, self._watch)

        self._spawn('startup')

    # --- device discovery (diagnostics) ---------------------------------
    def _detect_devices(self):
        found = []
        try:
            with open('/proc/bus/input/devices', 'r') as handle:
                blocks = handle.read().split('\n\n')
        except OSError:
            return found
        for block in blocks:
            name = ''
            js = None
            for line in block.splitlines():
                if line.startswith('N: ') and 'Name="' in line:
                    name = line.split('Name="', 1)[1].rstrip('"')
                elif line.startswith('H: ') and 'Handlers=' in line:
                    for handler in line.split('Handlers=', 1)[1].split():
                        if handler.startswith('js'):
                            js = '/dev/input/' + handler
            if js is not None:
                found.append((name, js))
        return found

    # --- child process management ---------------------------------------
    def _spawn(self, reason):
        devices = self._detect_devices()
        if devices:
            self.get_logger().info(
                'Joystick device(s) present: '
                + ', '.join('%s (%s)' % (n, p) for n, p in devices))
        else:
            self.get_logger().warn(
                'No joystick (js*) device found. Is the controller connected and '
                'is /dev/input mapped into the container?')

        # stdbuf keeps joy_node's stdout/stderr line-buffered so its "Opened
        # joystick" / error messages appear promptly rather than only on exit.
        cmd = [
            'stdbuf', '-oL', '-eL',
            'ros2', 'run', self.joy_package, self.joy_executable, '--ros-args',
            '-p', 'device_id:=%d' % int(self.device_id),
            '-p', 'deadzone:=%f' % float(self.deadzone),
            '-p', 'autorepeat_rate:=%f' % float(self.autorepeat_rate),
        ]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True)
        except FileNotFoundError as exc:
            self._proc = None
            self.get_logger().error('Could not start joy driver: %s' % exc)
            return

        self._exit_reported = False
        self._reader = threading.Thread(
            target=self._drain, args=(self._proc,), daemon=True)
        self._reader.start()
        self.get_logger().info('Started joy driver (%s): %s' % (reason, ' '.join(cmd)))

    def _drain(self, proc):
        try:
            for line in proc.stdout:
                line = line.rstrip()
                if line:
                    self.get_logger().info('[joy_node] ' + line)
        except (ValueError, OSError):
            pass

    def _kill(self):
        if self._proc is None or self._proc.poll() is not None:
            self._proc = None
            return
        try:
            pgid = os.getpgid(self._proc.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                self._proc.wait(timeout=2.0)
        except (ProcessLookupError, PermissionError):
            pass
        self._proc = None

    # --- ROS callbacks ---------------------------------------------------
    def _on_reconnect(self, _request, response):
        self.get_logger().info('Reconnect requested — restarting joy driver')
        self._kill()
        # Let the old driver fully release the device before reopening it.
        time.sleep(max(0.0, float(self.restart_delay)))
        self._spawn('manual reconnect')
        response.success = self._proc is not None
        response.message = ('Joy driver restarted' if response.success
                            else 'Failed to start joy driver')
        return response

    def _watch(self):
        # Report a dead driver once; do not auto-restart (that just races the
        # device). The user can press Reconnect to bring it back.
        if self._proc is None:
            return
        code = self._proc.poll()
        if code is not None and not self._exit_reported:
            self._exit_reported = True
            self.get_logger().error(
                'joy driver exited (code %s). Press Reconnect to restart it.' % code)

    def destroy_node(self):
        self._kill()
        super().destroy_node()


def main():
    rclpy.init()
    node = JoyManager()

    # Make SIGTERM behave like Ctrl-C so the child joy_node is never orphaned
    # (ros2 launch sends SIGINT first, but SIGTERM may follow).
    def _on_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # terminates the child joy_node
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
