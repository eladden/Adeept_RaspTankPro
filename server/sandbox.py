#!/usr/bin/env python3
"""
sandbox.py — Student programming sandbox for the RaspTankPro

HOW TO USE
----------
1. Open this file in Thonny (it should already be open at login).
2. Write your robot program inside the run() function below.
3. Press F5 (or the Run button) to execute your program.
4. Press the Stop button (or F2) at any time to stop the robot safely.

The web control server is stopped automatically when you press Run,
so all hardware is available to your program.

See STUDENT_GUIDE.md for the full API reference and examples.
"""

import subprocess
import time
import signal
import sys

# ---------------------------------------------------------------------------
# Stop the web server so we can use GPIO and the camera freely
# ---------------------------------------------------------------------------
subprocess.run(['sudo', 'pkill', '-f', 'webServer.py'], capture_output=True)
subprocess.run(['sudo', 'fuser', '-k', '/dev/video0'], capture_output=True)
time.sleep(3)   # Allow GPIO pins and camera to be released

# ---------------------------------------------------------------------------
# Create the robot object — this initialises all hardware
# ---------------------------------------------------------------------------
from robot_api import Robot

robot = Robot()
# If the robot drives backwards when forward() is called, the motors are
# connected in reverse. Fix it by passing reverse_motors=True:
# robot = Robot(reverse_motors=True)


# ---------------------------------------------------------------------------
# SIGTERM handler — ensures motors stop when Thonny's Stop button is pressed
# ---------------------------------------------------------------------------
def _sigterm_handler(signum, frame):
    robot.cleanup()
    sys.exit(0)

signal.signal(signal.SIGTERM, _sigterm_handler)


# ===========================================================================
# YOUR FUNCTIONS GO HERE
# ===========================================================================
"""
    Write your functions here like this:"""
def myfunc(parameterInA,paremeterInB):
    """
    Do something
    """
    parameterOutB = parameterInA
    parameterOutA = paremeterInB

    return parameterOutA, parameterOutB

# ===========================================================================
# YOUR RUN CODE GOES HERE
# ===========================================================================

def run():
    """
    Write your robot program in this function.

    The `robot` object gives you access to all motors, sensors, and servos.
    See STUDENT_GUIDE.md for the complete API reference.

    Tip: uncomment any of the examples below to try them out.
    """

    # --- Movement ---
    # robot.forward(speed=50, duration=2.0)   # Move forward for 2 seconds
    # robot.backward(speed=50, duration=1.0)  # Move backward for 1 second
    # robot.turn_left(speed=50, duration=0.5) # Pivot left for 0.5 seconds
    # robot.spin_right(speed=40, duration=1.0)# Spin right in place
    # robot.stop()                            # Stop immediately

    # --- Distance sensor ---
    # distance = robot.get_distance()
    # print(f"Obstacle at {distance:.2f} m")

    # --- Gyro & accelerometer ---
    # gyro = robot.get_gyro()
    # accel = robot.get_accel()
    # print(f"Gyro:  x={gyro['x']:.2f}  y={gyro['y']:.2f}  z={gyro['z']:.2f}")
    # print(f"Accel: x={accel['x']:.2f}  y={accel['y']:.2f}  z={accel['z']:.2f}")

    # --- Servos ---
    # robot.set_arm_rotation(-0.5) # Rotate arm left
    # robot.set_arm(0.8)           # Raise arm
    # robot.set_hand(0.5)          # Raise hand
    # robot.set_gripper(-1.0)      # Close gripper
    # robot.set_camera_tilt(0.5)   # Tilt camera up
    # robot.reset_servos()         # Return all servos to centre

    # --- Visual odometry ---
    # robot.start_odometry()
    # robot.forward(speed=40, duration=3.0)
    # x, y, z = robot.get_position()
    # print(f"Position: x={x:.2f}  y={y:.2f}  z={z:.2f}")
    # robot.stop_odometry()

    # --- Hardware tests ---
    # from robot_test import run_all_tests
    # run_all_tests(robot)

    pass  # Replace this line with your code


# ===========================================================================
# Do not edit below this line
# ===========================================================================

if __name__ == '__main__':
    try:
        run()
    finally:
        robot.cleanup()
