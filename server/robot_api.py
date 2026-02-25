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
    _pwm = Adafruit_PCA9685.PCA9685(0x40, busnum=1)
    _pwm.set_pwm_freq(50)
    _SERVO_AVAILABLE = True
except Exception as e:
    print(f"[robot_api] Servo init failed: {e}")
    _pwm = None
    _SERVO_AVAILABLE = False

try:
    from mpu6050 import mpu6050
    _imu = mpu6050(0x68)
    _IMU_AVAILABLE = True
except Exception as e:
    print(f"[robot_api] IMU init failed: {e}")
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
# picamera2 wrapper (Trixie / Bookworm — libcamera native API)
# ---------------------------------------------------------------------------

class _Picam2Capture:
    """
    VideoCapture-compatible wrapper around picamera2.

    Used automatically on Trixie/Bookworm where the Pi camera is only
    accessible via libcamera (V4L2 buffer allocation fails there).
    Raises ImportError on Buster where picamera2 is not installed,
    allowing the caller to fall back to cv2.VideoCapture.
    """
    def __init__(self, width=640, height=480):
        from picamera2 import Picamera2          # ImportError on Buster → caller falls back
        self._cam = Picamera2()
        cfg = self._cam.create_preview_configuration(
            main={"format": "BGR888", "size": (width, height)}
        )
        self._cam.configure(cfg)
        self._cam.start()
        self._opened = True

    def isOpened(self):
        return self._opened

    def read(self):
        try:
            frame = self._cam.capture_array()    # BGR numpy array
            return True, frame
        except Exception:
            return False, None

    def get(self, prop):      return 0
    def set(self, prop, val): return False

    def release(self):
        if self._opened:
            self._cam.stop()
            self._cam.close()
            self._opened = False


# ---------------------------------------------------------------------------
# Robot class
# ---------------------------------------------------------------------------

class Robot:
    """
    High-level interface to the RaspTankPro robot.

    Parameters
    ----------
    reverse_motors : bool
        Set to True if the robot drives backwards when forward() is called.
        This happens when the motor A/B connectors are swapped during assembly.
        Default is False.

    Example
    -------
    robot = Robot()                      # normal wiring
    robot = Robot(reverse_motors=True)   # motors connected backwards
    robot.forward(speed=50, duration=2.0)
    distance = robot.get_distance()
    robot.cleanup()
    """

    def __init__(self, reverse_motors=False):
        self._reverse_motors = reverse_motors
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

    # Direction helpers — swap strings when motors are wired backwards
    def _fwd(self):  return 'backward' if self._reverse_motors else 'forward'
    def _bwd(self):  return 'forward'  if self._reverse_motors else 'backward'
    def _spinL(self): return 'right'   if self._reverse_motors else 'left'
    def _spinR(self): return 'left'    if self._reverse_motors else 'right'

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
        move.move(speed, self._fwd(), 'no')
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
        move.move(speed, self._bwd(), 'no')
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
        move.move(speed, self._fwd(), 'left', radius=0)
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
        move.move(speed, self._fwd(), 'right', radius=0)
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
        move.move(speed, 'no', self._spinL())
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
        move.move(speed, 'no', self._spinR())
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

    def start_odometry(self, focal_length=537.0, pp=(320.0, 240.0), scale=1.0, show_debug=False):
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
        show_debug : bool
            If True, opens two OpenCV windows while odometry is running:
            a camera feed with tracked feature points and a 2-D trajectory
            map. Useful for verifying the camera is working and the robot's
            estimated path makes sense. Default False.

        Raises
        ------
        RuntimeError
            If the camera cannot be opened.
        """
        if self._odometry_running:
            return

        self._show_debug = show_debug
        if show_debug:
            import numpy as np
            self._traj_canvas = np.zeros((500, 500, 3), dtype=np.uint8)
            self._traj_origin = (250, 250)

        import cv2

        # Step 1: open with OpenCV VideoCapture (primary path — works on Buster)
        for attempt in range(1, 6):
            self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                self._cap = cv2.VideoCapture(0)
            if self._cap.isOpened():
                break
            self._cap.release()
            print(f"[odometry] Camera busy, retrying ({attempt}/5)...")
            time.sleep(1)
        if not self._cap.isOpened():
            raise RuntimeError(
                "Cannot open camera. Is another program using it?"
            )

        # Step 2: verify the device actually delivers frames.
        # On Trixie/Bookworm, isOpened()=True but V4L2 buffer allocation
        # silently fails, so all reads return False immediately.
        frames_ok = False
        for _ in range(5):
            ret, _ = self._cap.read()
            if ret:
                frames_ok = True
                break
            time.sleep(0.05)

        if not frames_ok:
            # V4L2 can't deliver frames — fall back to picamera2 (Trixie/Bookworm)
            self._cap.release()
            self._cap = None
            try:
                self._cap = _Picam2Capture(width=640, height=480)
                print("[odometry] Falling back to picamera2 (libcamera native)")
            except ImportError:
                raise RuntimeError(
                    "Camera opens but delivers no frames. "
                    "On Trixie/Bookworm, install: sudo apt install python3-picamera2"
                )
            except Exception as e:
                raise RuntimeError(f"Camera not available: {e}")

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
                if self._show_debug:
                    self._draw_debug()
            except RuntimeError as e:
                print(f"[odometry] Stopped: {e}")
                self._odometry_running = False
                break
            except Exception:
                pass

    def _draw_debug(self):
        """Draw live camera feed + 2-D trajectory when show_debug=True."""
        import cv2 as _cv2
        # Camera feed with tracked feature points
        frame = self._vo.colorframe.copy()
        if hasattr(self._vo, 'good_new') and len(self._vo.good_new) > 0:
            for pt in self._vo.good_new:
                _cv2.circle(frame, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
        _cv2.imshow('Odometry - Camera', frame)

        # 2-D top-down trajectory (x = lateral, z = forward = up on canvas)
        x, _, z = self._position
        scale = 20  # pixels per unit
        cx = self._traj_origin[0] + int(x * scale)
        cz = self._traj_origin[1] - int(z * scale)
        cx = max(0, min(499, cx))
        cz = max(0, min(499, cz))
        _cv2.circle(self._traj_canvas, (cx, cz), 2, (0, 200, 255), -1)
        _cv2.imshow('Odometry - Trajectory', self._traj_canvas)

        _cv2.waitKey(1)

    def stop_odometry(self):
        """Stop visual odometry and release the camera."""
        self._odometry_running = False
        if self._odometry_thread is not None:
            self._odometry_thread.join(timeout=2.0)
            self._odometry_thread = None
        if getattr(self, '_show_debug', False):
            import cv2
            cv2.destroyAllWindows()
            cv2.waitKey(1)
            self._show_debug = False
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
