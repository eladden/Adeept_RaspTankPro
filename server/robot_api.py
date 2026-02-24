#!/usr/bin/env python3
"""
robot_api.py — High-level Robot API for the Adeept RaspTankPro

Provides a clean, beginner-friendly interface to all robot hardware.
Students import this module in their sandbox programs.

See STUDENT_GUIDE.md for full documentation and examples.
"""

import time
import threading

# ---------------------------------------------------------------------------
# Low-level hardware imports (available only on the Raspberry Pi)
# ---------------------------------------------------------------------------
import move
import ultra

try:
    import Adafruit_PCA9685
    _pwm = Adafruit_PCA9685.PCA9685()
    _pwm.set_pwm_freq(50)
    _SERVO_AVAILABLE = True
except Exception:
    _pwm = None
    _SERVO_AVAILABLE = False

try:
    from mpu6050 import mpu6050
    _imu = mpu6050(0x68)
    _IMU_AVAILABLE = True
except Exception:
    _imu = None
    _IMU_AVAILABLE = False

from monovideoodometery import MonoVideoOdometeryFromCam

# ---------------------------------------------------------------------------
# Servo channel definitions (matching servo.py)
# ---------------------------------------------------------------------------
# channel: (init_pwm, min_pwm, max_pwm, direction)
#   direction=1  → position +1.0 maps to max_pwm
#   direction=-1 → position +1.0 maps to min_pwm
_SERVO_CFG = {
    'arm_rotation': (0, 300, 150, 450,  1),   # PWM0: turn arm left-right
    'arm':          (1, 300, 160, 480,  1),   # PWM1: arm up-down
    'hand':         (2, 300, 100, 500, -1),   # PWM2: hand up-down
    'gripper':      (3, 300, 100, 500,  1),   # PWM3: gripper open-close
    'camera_tilt':  (4, 300, 100, 500, -1),   # PWM4: tilt camera up-down
}


def _position_to_pwm(position, init, lo, hi, direction):
    """Map position [-1.0, +1.0] to a PWM value."""
    position = max(-1.0, min(1.0, float(position)))
    if direction == 1:
        center = init
        if position >= 0:
            pwm = center + position * (hi - center)
        else:
            pwm = center + position * (center - lo)
    else:
        center = init
        if position >= 0:
            pwm = center - position * (center - lo)
        else:
            pwm = center - position * (hi - center)
    return int(round(pwm))


# ---------------------------------------------------------------------------
# Robot class
# ---------------------------------------------------------------------------

class Robot:
    """
    High-level interface to the RaspTankPro robot.

    Example
    -------
    robot = Robot()
    robot.forward(speed=50, duration=2.0)
    distance = robot.get_distance()
    robot.cleanup()
    """

    def __init__(self):
        move.setup()
        self._odometry_thread = None
        self._odometry_running = False
        self._position = (0.0, 0.0, 0.0)
        self._position_lock = threading.Lock()
        self._vo = None
        self._cap = None

        # Reset servos to center on startup
        if _SERVO_AVAILABLE:
            self.reset_servos()

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def forward(self, speed=50, duration=None):
        """
        Move the robot forward.

        Parameters
        ----------
        speed : int
            Motor speed, 0–100 (default 50).
        duration : float or None
            Seconds to move. If None, moves until stop() is called.
        """
        move.move(speed, 'forward', 'no')
        if duration is not None:
            time.sleep(duration)
            move.motorStop()

    def backward(self, speed=50, duration=None):
        """
        Move the robot backward.

        Parameters
        ----------
        speed : int
            Motor speed, 0–100 (default 50).
        duration : float or None
            Seconds to move. If None, moves until stop() is called.
        """
        move.move(speed, 'backward', 'no')
        if duration is not None:
            time.sleep(duration)
            move.motorStop()

    def turn_left(self, speed=50, duration=None):
        """
        Pivot left (right wheel drives forward, left wheel stops).

        Parameters
        ----------
        speed : int
            Motor speed, 0–100 (default 50).
        duration : float or None
            Seconds to turn. If None, turns until stop() is called.
        """
        move.move(speed, 'forward', 'left', radius=0)
        if duration is not None:
            time.sleep(duration)
            move.motorStop()

    def turn_right(self, speed=50, duration=None):
        """
        Pivot right (left wheel drives forward, right wheel stops).

        Parameters
        ----------
        speed : int
            Motor speed, 0–100 (default 50).
        duration : float or None
            Seconds to turn. If None, turns until stop() is called.
        """
        move.move(speed, 'forward', 'right', radius=0)
        if duration is not None:
            time.sleep(duration)
            move.motorStop()

    def spin_left(self, speed=50, duration=None):
        """
        Spin in place counter-clockwise (wheels turn in opposite directions).

        Parameters
        ----------
        speed : int
            Motor speed, 0–100 (default 50).
        duration : float or None
            Seconds to spin. If None, spins until stop() is called.
        """
        move.move(speed, 'no', 'left')
        if duration is not None:
            time.sleep(duration)
            move.motorStop()

    def spin_right(self, speed=50, duration=None):
        """
        Spin in place clockwise (wheels turn in opposite directions).

        Parameters
        ----------
        speed : int
            Motor speed, 0–100 (default 50).
        duration : float or None
            Seconds to spin. If None, spins until stop() is called.
        """
        move.move(speed, 'no', 'right')
        if duration is not None:
            time.sleep(duration)
            move.motorStop()

    def stop(self):
        """Stop all motors immediately."""
        move.motorStop()

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def get_distance(self):
        """
        Read the ultrasonic distance sensor.

        Returns
        -------
        float
            Distance to the nearest obstacle in metres.
        """
        return ultra.checkdist()

    def get_gyro(self):
        """
        Read angular velocity from the MPU6050 gyroscope.

        Returns
        -------
        dict
            {'x': float, 'y': float, 'z': float} in degrees per second.
            Returns {'x': 0, 'y': 0, 'z': 0} if the sensor is not connected.
        """
        if not _IMU_AVAILABLE:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        return _imu.get_gyro_data()

    def get_accel(self):
        """
        Read acceleration from the MPU6050 accelerometer.

        Returns
        -------
        dict
            {'x': float, 'y': float, 'z': float} in g (9.81 m/s²).
            Returns {'x': 0, 'y': 0, 'z': 1} if the sensor is not connected.
        """
        if not _IMU_AVAILABLE:
            return {'x': 0.0, 'y': 0.0, 'z': 1.0}
        return _imu.get_accel_data()

    # ------------------------------------------------------------------
    # Servos
    # ------------------------------------------------------------------

    def _set_servo(self, name, position):
        if not _SERVO_AVAILABLE:
            return
        ch, init, lo, hi, direction = _SERVO_CFG[name]
        pwm_val = _position_to_pwm(position, init, lo, hi, direction)
        _pwm.set_pwm(ch, 0, pwm_val)

    def set_arm_rotation(self, position):
        """
        Rotate the arm left or right (turret rotation).

        Parameters
        ----------
        position : float
            -1.0 = full left, 0.0 = centre, +1.0 = full right.
        """
        self._set_servo('arm_rotation', position)

    def set_arm(self, position):
        """
        Move the arm joint up or down.

        Parameters
        ----------
        position : float
            -1.0 = fully down, 0.0 = centre, +1.0 = fully up.
        """
        self._set_servo('arm', position)

    def set_hand(self, position):
        """
        Move the hand (wrist/forearm) up or down.

        Parameters
        ----------
        position : float
            -1.0 = fully down, 0.0 = centre, +1.0 = fully up.
        """
        self._set_servo('hand', position)

    def set_gripper(self, position):
        """
        Open or close the gripper.

        Parameters
        ----------
        position : float
            -1.0 = fully closed, 0.0 = centre, +1.0 = fully open.
        """
        self._set_servo('gripper', position)

    def set_camera_tilt(self, position):
        """
        Tilt the camera up or down.

        Parameters
        ----------
        position : float
            -1.0 = full down, 0.0 = centre, +1.0 = full up.
        """
        self._set_servo('camera_tilt', position)

    def reset_servos(self):
        """Return all servos to their centre position."""
        if not _SERVO_AVAILABLE:
            return
        for name in _SERVO_CFG:
            ch, init, lo, hi, direction = _SERVO_CFG[name]
            _pwm.set_pwm(ch, 0, init)

    # ------------------------------------------------------------------
    # Visual Odometry
    # ------------------------------------------------------------------

    def start_odometry(self, focal_length=537.0, pp=(320.0, 240.0), scale=1.0):
        """
        Start visual odometry in the background.

        The robot's position is estimated from the camera feed using
        monocular visual odometry (FAST features + Lucas-Kanade optical flow).

        Note: Monocular odometry cannot determine absolute scale without
        calibration. The returned x/y/z values are in relative units.
        Call reset_position() to set the current location as the new origin.

        Parameters
        ----------
        focal_length : float
            Camera focal length in pixels (default 537.0 for the Pi camera
            at 640×480).
        pp : tuple
            Principal point (cx, cy) in pixels (default (320.0, 240.0)).
        scale : float
            Absolute scale factor applied to each translation step
            (default 1.0).

        Raises
        ------
        RuntimeError
            If the camera cannot be opened.
        """
        if self._odometry_running:
            return

        import cv2
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            raise RuntimeError(
                "Cannot open camera. Is another program using it?"
            )

        self._vo = MonoVideoOdometeryFromCam(
            self._cap,
            focal_length=focal_length,
            pp=pp,
            absolute_scale=scale,
        )
        with self._position_lock:
            coords = self._vo.get_mono_coordinates()
            self._position = (float(coords[0]), float(coords[1]), float(coords[2]))

        self._odometry_running = True
        self._odometry_thread = threading.Thread(
            target=self._odometry_loop, daemon=True
        )
        self._odometry_thread.start()

    def _odometry_loop(self):
        while self._odometry_running:
            try:
                self._vo.process_frame()
                coords = self._vo.get_mono_coordinates()
                with self._position_lock:
                    self._position = (
                        float(coords[0]),
                        float(coords[1]),
                        float(coords[2]),
                    )
            except Exception:
                pass

    def stop_odometry(self):
        """Stop visual odometry and release the camera."""
        self._odometry_running = False
        if self._odometry_thread is not None:
            self._odometry_thread.join(timeout=2.0)
            self._odometry_thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._vo = None

    def get_position(self):
        """
        Return the latest estimated position from visual odometry.

        Returns
        -------
        tuple
            (x, y, z) floats in relative units. The origin is the robot's
            position when start_odometry() was called (or reset_position()).

        Raises
        ------
        RuntimeError
            If start_odometry() has not been called.
        """
        if not self._odometry_running:
            raise RuntimeError(
                "Odometry is not running. Call start_odometry() first."
            )
        with self._position_lock:
            return self._position

    def reset_position(self):
        """
        Reset the odometry origin to the current position.

        Restarts the visual odometry instance so that the current camera
        view becomes (0, 0, 0).

        Raises
        ------
        RuntimeError
            If start_odometry() has not been called.
        """
        if not self._odometry_running or self._cap is None:
            raise RuntimeError(
                "Odometry is not running. Call start_odometry() first."
            )
        # Pause the loop, recreate the VO instance from the same camera feed
        self._odometry_running = False
        if self._odometry_thread is not None:
            self._odometry_thread.join(timeout=2.0)

        self._vo = MonoVideoOdometeryFromCam(
            self._cap,
            focal_length=self._vo.focal,
            pp=self._vo.pp,
            absolute_scale=self._vo.absolute_scale,
        )
        with self._position_lock:
            self._position = (0.0, 0.0, 0.0)

        self._odometry_running = True
        self._odometry_thread = threading.Thread(
            target=self._odometry_loop, daemon=True
        )
        self._odometry_thread.start()

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def wait(self, seconds):
        """
        Pause execution for the given number of seconds.

        Parameters
        ----------
        seconds : float
            Time to wait.
        """
        time.sleep(seconds)

    def cleanup(self):
        """
        Stop motors, stop odometry, and release all hardware resources.

        Called automatically when using the Robot as a context manager, or
        by sandbox.py's SIGTERM handler.
        """
        move.motorStop()
        self.stop_odometry()
        move.destroy()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
        return False
