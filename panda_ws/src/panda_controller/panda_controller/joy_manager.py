#!/usr/bin/env python3
"""Supervisor for the joystick driver.

The stock joy_node keeps the handle it opened at startup, so if the controller
is unplugged and replugged it never re-establishes communication. This node owns
a single joy_node child process and restarts it cleanly on demand (the
/joystick/reconnect service, wired to the dashboard button).

A plain kill+respawn is not enough: under load (e.g. the full simulation) the
fresh joy_node sometimes comes up before the device has settled and silently
fails to open it. So each (re)start is *verified*: the manager watches joy_node's
output for "Opened joystick" and, if it does not appear within open_timeout,
restarts again — up to max_open_retries times. This is a bounded retry tied to an
explicit start/reconnect, not a perpetual watchdog.

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
        # device handle is released before the fresh joy_node opens it.
        self.restart_delay = self.declare_parameter('restart_delay', 1.5).value
        # If joy_node does not report "Opened joystick" within this many seconds,
        # the start is considered failed and retried (up to max_open_retries).
        self.open_timeout = self.declare_parameter('open_timeout', 3.0).value
        self.max_open_retries = self.declare_parameter('max_open_retries', 4).value

        self._proc = None
        self._reader = None
        self._joy_exe = None
        self._opened = False
        self._spawn_time = 0.0
        self._retries_left = 0
        self._failure_reported = False

        self.reconnect_srv = self.create_service(
            Trigger, '/joystick/reconnect', self._on_reconnect)
        self.create_timer(1.0, self._watch)

        self._start_driver('startup', delay=False)

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

    def _resolve_joy_exe(self):
        """Locate joy_node so we can run it directly (skipping the 'ros2 run'
        wrapper, which spawns it as a grandchild and hides its output/signals)."""
        if self._joy_exe is not None:
            return self._joy_exe
        try:
            prefix = subprocess.check_output(
                ['ros2', 'pkg', 'prefix', self.joy_package], text=True).strip()
            exe = os.path.join(prefix, 'lib', self.joy_package, self.joy_executable)
            if os.path.exists(exe):
                self._joy_exe = exe
        except (subprocess.SubprocessError, OSError):
            self._joy_exe = None
        return self._joy_exe

    # --- driver lifecycle -----------------------------------------------
    def _start_driver(self, reason, delay=True):
        """Start (or restart) the driver with a fresh retry budget."""
        self._retries_left = int(self.max_open_retries)
        self._failure_reported = False
        self._respawn(reason, delay)

    def _respawn(self, reason, delay=True):
        self._kill()
        if delay:
            time.sleep(max(0.0, float(self.restart_delay)))
        self._spawn(reason)

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

        node_args = [
            '--ros-args',
            '-p', 'device_id:=%d' % int(self.device_id),
            '-p', 'deadzone:=%f' % float(self.deadzone),
            '-p', 'autorepeat_rate:=%f' % float(self.autorepeat_rate),
        ]
        exe = self._resolve_joy_exe()
        if exe is not None:
            cmd = ['stdbuf', '-oL', '-eL', exe] + node_args
        else:
            cmd = ['stdbuf', '-oL', '-eL', 'ros2', 'run',
                   self.joy_package, self.joy_executable] + node_args
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, start_new_session=True)
        except FileNotFoundError as exc:
            self._proc = None
            self.get_logger().error('Could not start joy driver: %s' % exc)
            return

        self._opened = False
        self._spawn_time = time.monotonic()
        self._reader = threading.Thread(
            target=self._drain, args=(self._proc,), daemon=True)
        self._reader.start()
        self.get_logger().info('Started joy driver (%s): %s' % (reason, ' '.join(cmd)))

    def _drain(self, proc):
        try:
            for line in proc.stdout:
                line = line.rstrip()
                if not line:
                    continue
                if 'Opened joystick' in line:
                    self._opened = True
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
        self._start_driver('manual reconnect')
        # Report the current outcome; the verify/retry loop may still be running.
        response.success = self._proc is not None
        response.message = ('Joy driver restarting…' if response.success
                            else 'Failed to start joy driver')
        return response

    def _watch(self):
        """Verify the running driver actually opened a joystick; retry if not."""
        if self._proc is None:
            return
        now = time.monotonic()

        code = self._proc.poll()
        if code is not None:
            # Process died before opening anything.
            if self._retries_left > 0:
                self._retries_left -= 1
                self.get_logger().warn(
                    'joy driver exited (code %s) — retrying (%d left)'
                    % (code, self._retries_left))
                self._respawn('retry after exit')
            elif not self._failure_reported:
                self._failure_reported = True
                self.get_logger().error(
                    'joy driver exited (code %s) and retries exhausted. '
                    'Press Reconnect once the controller is back.' % code)
            return

        if self._opened:
            return  # healthy

        # Alive but no joystick opened yet.
        if (now - self._spawn_time) <= self.open_timeout:
            return  # still within the grace window
        if self._retries_left > 0:
            self._retries_left -= 1
            self.get_logger().warn(
                'joy_node did not open a joystick within %.0fs — retrying (%d left)'
                % (self.open_timeout, self._retries_left))
            self._respawn('retry (no joystick opened)')
        elif not self._failure_reported:
            self._failure_reported = True
            self.get_logger().error(
                'Could not open a joystick after %d attempts. Check the '
                'controller is awake and js0 is present, then press Reconnect.'
                % self.max_open_retries)

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
