#!/usr/bin/env python3
"""
robot_test.py — Hardware verification for the RaspTankPro

Tests every hardware component and prints PASS / FAIL for each.

HOW TO USE
----------
Option A — from sandbox.py (recommended):
    In sandbox.py, replace the run() body with:

        from robot_test import run_all_tests
        run_all_tests(robot)

Option B — directly in Thonny:
    Switch Thonny's file to robot_test.py and press F5.
    (Standalone mode stops the web server automatically before running.)

Everything printed is also written to /home/pi/robot_test.log (appended,
one timestamped block per run). Every line is forced onto the SD card the
moment it is printed, so if power dies mid-test the log's last line shows
exactly which test was running.
"""

import os
import sys
import time
import datetime

LOG_FILE = '/home/pi/robot_test.log'


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

class _Tee:
    """Write to the terminal and the log file at once; flush + fsync every
    write so the log survives a sudden power cut."""

    def __init__(self, stream, logfile):
        self._stream = stream
        self._log = logfile

    def write(self, text):
        self._stream.write(text)
        self._stream.flush()
        try:
            self._log.write(text)
            self._log.flush()
            os.fsync(self._log.fileno())
        except (OSError, ValueError):
            pass  # never let logging kill a test

    def flush(self):
        self._stream.flush()
        try:
            self._log.flush()
        except (OSError, ValueError):
            pass


def _ok(label):
    print(f"  [ PASS ] {label}")


def _fail(label, reason=""):
    suffix = f" — {reason}" if reason else ""
    print(f"  [ FAIL ] {label}{suffix}")


def _section(title):
    print(f"\n{'='*50}")
    print(f"  {title}")
    print('='*50)


# ---------------------------------------------------------------------------
# Individual tests
# ---------------------------------------------------------------------------

def _test_motors(robot):
    _section("Motors")
    try:
        print("  Moving forward 1 s …")
        robot.forward(speed=50, duration=1.0)
        _ok("forward")
    except Exception as e:
        _fail("forward", str(e))

    try:
        print("  Moving backward 1 s …")
        robot.backward(speed=50, duration=1.0)
        _ok("backward")
    except Exception as e:
        _fail("backward", str(e))

    try:
        print("  Spinning left 0.5 s …")
        robot.spin_left(speed=50, duration=0.5)
        _ok("spin_left")
    except Exception as e:
        _fail("spin_left", str(e))

    try:
        print("  Spinning right 0.5 s …")
        robot.spin_right(speed=50, duration=0.5)
        _ok("spin_right")
    except Exception as e:
        _fail("spin_right", str(e))

    robot.stop()


def _test_distance(robot):
    _section("Ultrasonic Distance Sensor")
    readings = []
    for i in range(3):
        try:
            d = robot.get_distance()
            readings.append(d)
            print(f"  Reading {i+1}: {d:.3f} m")
            time.sleep(0.5)
        except Exception as e:
            _fail(f"reading {i+1}", str(e))
            return

    if all(r > 0 for r in readings):
        _ok(f"3 readings — average {sum(readings)/len(readings):.3f} m")
    else:
        _fail("readings", "one or more readings were zero or negative")


def _test_gyro(robot):
    _section("Gyroscope (MPU6050)")
    try:
        gyro = robot.get_gyro()
        print(f"  x={gyro['x']:.3f}  y={gyro['y']:.3f}  z={gyro['z']:.3f}  °/s")
        if all(k in gyro for k in ('x', 'y', 'z')):
            _ok("gyro data received")
        else:
            _fail("gyro", "unexpected dict format")
    except Exception as e:
        _fail("gyro", str(e))


def _test_accel(robot):
    _section("Accelerometer (MPU6050)")
    try:
        accel = robot.get_accel()
        print(f"  x={accel['x']:.3f}  y={accel['y']:.3f}  z={accel['z']:.3f}  g")
        if all(k in accel for k in ('x', 'y', 'z')):
            _ok("accel data received")
        else:
            _fail("accel", "unexpected dict format")
    except Exception as e:
        _fail("accel", str(e))


def _test_servos(robot, pause=2.0):
    _section("Servos")
    servos = [
        ("arm_rotation (left-right)", robot.set_arm_rotation),
        ("arm (up-down)",             robot.set_arm),
        ("hand (up-down)",            robot.set_hand),
        ("gripper (open-close)",      robot.set_gripper),
        ("camera_tilt (up-down)",     robot.set_camera_tilt),
    ]
    for name, setter in servos:
        # the marker hits the SD card before the servo moves, so a power
        # cut mid-movement leaves this as the log's last line
        print(f"  >>> testing servo: {name}")
        time.sleep(pause)
        try:
            print(f"  Sweeping {name}: -1 → +1 → 0 …")
            setter(-1.0)   # servo ramps smoothly; call returns on arrival
            time.sleep(0.4)
            setter(1.0)
            time.sleep(0.4)
            setter(0.0)
            time.sleep(0.2)
            _ok(name)
        except Exception as e:
            _fail(name, str(e))

    robot.reset_servos()


def _test_odometry(robot, show_debug=True):
    _section("Visual Odometry")
    try:
        print("  Starting odometry …")
        robot.start_odometry(show_debug=show_debug)
        _ok("start_odometry")
    except Exception as e:
        _fail("start_odometry", str(e))
        return

    try:
        print("  Driving forward 3 s to accumulate position …")
        robot.forward(speed=40, duration=3.0)
        x, y, z = robot.get_position()
        print(f"  Estimated position: x={x:.3f}  y={y:.3f}  z={z:.3f}")
        _ok("get_position returned a 3-tuple")
    except Exception as e:
        _fail("get_position", str(e))

    try:
        robot.stop_odometry()
        _ok("stop_odometry")
    except Exception as e:
        _fail("stop_odometry", str(e))


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def run_all_tests(robot, show_debug=True, pause=2.0, log_file=None):
    """
    Run all hardware tests using the given Robot instance.

    Call this from sandbox.py:

        from robot_test import run_all_tests
        run_all_tests(robot)

    Parameters
    ----------
    robot : robot_api.Robot
        The already-initialised Robot object from sandbox.py.
    show_debug : bool
        If True (default), opens live camera and trajectory windows during
        the odometry test. Pass False to suppress them.
    pause : float
        Seconds to wait before each test (and each individual servo),
        so you can watch what is about to happen. Default 2.0.
    log_file : str or None
        Where to also write everything printed. Default
        /home/pi/robot_test.log (appended; falls back to a file next to
        robot_test.py if that path is not writable).
    """
    if log_file is None:
        log_file = LOG_FILE
    try:
        log = open(log_file, 'a')
    except OSError:
        log_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'robot_test.log')
        log = open(log_file, 'a')

    real_out, real_err = sys.stdout, sys.stderr
    sys.stdout = _Tee(real_out, log)
    sys.stderr = _Tee(real_err, log)
    try:
        stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print("\n" + "#"*50)
        print(f"  RaspTankPro Hardware Test — {stamp}")
        print(f"  (log: {log_file})")
        print("#"*50)

        tests = [
            ("Motors",            lambda: _test_motors(robot)),
            ("Ultrasonic sensor", lambda: _test_distance(robot)),
            ("Gyroscope",         lambda: _test_gyro(robot)),
            ("Accelerometer",     lambda: _test_accel(robot)),
            ("Servos",            lambda: _test_servos(robot, pause=pause)),
            ("Visual odometry",   lambda: _test_odometry(robot, show_debug=show_debug)),
        ]
        for title, test_fn in tests:
            print(f"\n>>> NEXT TEST: {title}  (starting in {pause:.0f} s)")
            time.sleep(pause)
            test_fn()

        print("\n" + "#"*50)
        print("  Test complete. Review any [ FAIL ] lines above.")
        print(f"  Full output saved to {log_file}")
        print("#"*50 + "\n")
    finally:
        sys.stdout, sys.stderr = real_out, real_err
        log.close()


# ---------------------------------------------------------------------------
# Standalone mode (open this file in Thonny and press F5)
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import os
    import subprocess
    import signal
    import sys

    subprocess.run(['sudo', 'pkill', '-f', 'webServer.py'], capture_output=True)
    time.sleep(1)

    from robot_api import Robot
    robot = Robot()

    def _sigterm_handler(signum, frame):
        robot.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        run_all_tests(robot)
    finally:
        robot.cleanup()
    os._exit(0)
