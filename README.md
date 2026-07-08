# Adeept RaspTankPro — Autonomous Robots Course

Software for the [Adeept RaspTank Pro](https://www.adeept.com/rasptank-pro_p0117.html) robot, used in an autonomous robots course. The robot runs on a Raspberry Pi and provides a web-based remote control interface as well as a Python sandbox environment for student programming.

## Hardware

| Component | Details |
|-----------|---------|
| Compute | Raspberry Pi |
| Drive | Two DC motors (differential drive) |
| Arm | 5 servos: arm rotation (left-right), arm (up-down), hand (up-down), gripper (open-close), camera tilt (up-down) |
| Sensors | Ultrasonic distance sensor, MPU6050 gyroscope + accelerometer |
| Camera | Raspberry Pi camera (live stream + visual odometry) |
| Connectivity | Wi-Fi hotspot (AP mode) |

## Repository Structure

```
server/          Low-level hardware drivers and web control server
  robot_api.py   ← High-level student API (wraps all hardware)
  sandbox.py     ← Student programming template (edit run() and press F5)
  robot_test.py  ← Hardware verification script
  webServer.py   ← Web control server (starts automatically on boot)
  move.py        Motor control
  servo.py       Servo control
  ultra.py       Ultrasonic sensor
  ...

Monocular-Video-Odometery/   Visual odometry (git submodule)
setup.py                     Robot installation script
setup_sandbox.py             One-time sandbox environment setup
STUDENT_GUIDE.md             Student programming guide (English)
STUDENT_GUIDE_HE.md          Student programming guide (Hebrew)
```

## Getting Started

### First-time robot setup

1. Clone this repository onto the robot:
   ```bash
   git clone --recursive https://github.com/eladden/Adeept_RaspTankPro.git
   ```
   The `--recursive` flag also clones the visual odometry submodule.

2. Run the installation script (installs all dependencies and configures autostart).
   It asks for the robot's number — the robot is then named `Robot<X>` (Wi-Fi hotspot
   name and OLED display). The script **reboots the robot when it finishes**:
   ```bash
   sudo python3 setup.py
   ```

3. After the reboot, run the sandbox setup script once to configure Thonny IDE autostart and the emergency-stop desktop shortcut, then reboot again:
   ```bash
   sudo python3 setup_sandbox.py
   sudo reboot
   ```

4. Done. The web control server starts automatically and Thonny opens `sandbox.py`.

### Normal operation (web control mode)

- If the robot finds a known Wi-Fi network it joins it (its IP is shown on the OLED screen).
  Otherwise it creates its own hotspot: network name **Robot\<X\>** (the robot's number),
  password **12345678**, robot address **192.168.12.1**.
- Open a browser and go to **http://192.168.12.1:5000** (or `http://<robot-ip>:5000` on a
  shared network). The page shows the live camera and the driving controls.

### Updating an installed robot

```bash
cd Adeept_RaspTankPro
git pull
git submodule update --init --recursive   # plain 'git pull' does NOT update the submodule
```

To re-run the installer after an update (refreshes packages and the autostart services),
first stop the running services, then install:

```bash
sudo python3 pre-resetup.py   # only needed before RE-running setup.py
sudo python3 setup.py         # reboots when done
sudo python3 setup_sandbox.py # after the reboot, then reboot again
```

Note for older (Buster) robots: during `setup.py` a few packages will fail to install with
red error messages (e.g. `python3-picamera2` does not exist for Buster). This is expected and
harmless — the installer retries and continues, and those packages are not needed on Buster.

### Student sandbox mode

Students work directly at the robot (keyboard + monitor):

1. **Thonny IDE opens automatically** at desktop login with `sandbox.py` loaded.
2. **Edit the `run()` function** in `sandbox.py` with your robot program.
3. **Press F5** — the web server stops automatically and your code runs.
4. **Press Stop (F2)** at any time to safely stop the robot.
5. **Reboot** to return to web control mode.

> **Emergency stop:** Double-click the **EMERGENCY STOP** icon on the desktop to kill the running program and stop all motors immediately.

### Student guide

Full API reference, examples, and troubleshooting:
- English: [STUDENT_GUIDE.md](STUDENT_GUIDE.md)
- Hebrew: [STUDENT_GUIDE_HE.md](STUDENT_GUIDE_HE.md)

## Visual Odometry

The `Monocular-Video-Odometery/` folder is a git submodule. If you cloned without `--recursive`, fetch it with:

```bash
git submodule update --init
```

Position tracking is available via the sandbox API:

```python
robot.start_odometry()
robot.forward(speed=40, duration=3.0)
x, y, z = robot.get_position()   # relative units
robot.stop_odometry()
```

`start_odometry(show_debug=True)` opens live debug windows — this requires a monitor
connected to the robot (with no display the windows are silently skipped).

## Hardware Verification

To test that all hardware components are working, add this to `sandbox.py` and press F5:

```python
from robot_test import run_all_tests

def run():
    run_all_tests(robot)
```
