#!/usr/bin/env python3
"""
setup_sandbox.py — One-time setup for the student sandbox environment

Run this script once on each robot to configure:
  1. Thonny IDE opens sandbox.py automatically when the desktop starts
  2. An "EMERGENCY STOP" shortcut is placed on the desktop
  3. /dev/mem access for the gpio group (needed for NeoPixel LEDs without sudo)

Usage (run from the project root):
    sudo python3 setup_sandbox.py

After running, reboot the robot. From then on, Thonny will open sandbox.py
every time a student logs in.
"""

import os
import subprocess
import sys
import stat

# ---------------------------------------------------------------------------
# Resolve paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
SANDBOX_PATH = os.path.join(SCRIPT_DIR, 'server', 'sandbox.py')

HOME = os.path.expanduser('~')
AUTOSTART_DIR = os.path.join(HOME, '.config', 'autostart')
DESKTOP_DIR = os.path.join(HOME, 'Desktop')

THONNY_DESKTOP_FILE = os.path.join(AUTOSTART_DIR, 'sandbox_thonny.desktop')
ESTOP_DESKTOP_FILE = os.path.join(DESKTOP_DIR, 'EMERGENCY_STOP.desktop')

# ---------------------------------------------------------------------------
# Create autostart directory if it doesn't exist
# ---------------------------------------------------------------------------
os.makedirs(AUTOSTART_DIR, exist_ok=True)
os.makedirs(DESKTOP_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# 1. Thonny autostart — opens sandbox.py on desktop login
# ---------------------------------------------------------------------------
thonny_desktop_content = f"""\
[Desktop Entry]
Type=Application
Name=Robot Sandbox (Thonny)
Comment=Open the student sandbox in Thonny IDE
Exec=thonny {SANDBOX_PATH}
Terminal=false
StartupNotify=false
"""

with open(THONNY_DESKTOP_FILE, 'w') as f:
    f.write(thonny_desktop_content)

os.chmod(THONNY_DESKTOP_FILE, stat.S_IRWXU | stat.S_IRGRP | stat.S_IROTH)
print(f"[OK] Thonny autostart configured: {THONNY_DESKTOP_FILE}")

# ---------------------------------------------------------------------------
# 2. Emergency Stop desktop shortcut
# ---------------------------------------------------------------------------
estop_desktop_content = """\
[Desktop Entry]
Type=Application
Name=EMERGENCY STOP
Comment=Immediately stop all robot movement
Exec=sudo pkill -f sandbox.py
Terminal=false
StartupNotify=false
Icon=gtk-stop
"""

with open(ESTOP_DESKTOP_FILE, 'w') as f:
    f.write(estop_desktop_content)

os.chmod(ESTOP_DESKTOP_FILE, stat.S_IRWXU | stat.S_IRGRP | stat.S_IROTH)
print(f"[OK] Emergency Stop shortcut created: {ESTOP_DESKTOP_FILE}")

# ---------------------------------------------------------------------------
# 3. /dev/mem udev rule — allows NeoPixel LEDs without sudo
#    rpi_ws281x uses DMA via /dev/mem; pi is in the gpio group by default.
# ---------------------------------------------------------------------------
UDEV_RULE_FILE = '/etc/udev/rules.d/99-rpi-mem.rules'
UDEV_RULE = 'KERNEL=="mem", GROUP="gpio", MODE="0660"\n'

with open(UDEV_RULE_FILE, 'w') as f:
    f.write(UDEV_RULE)
subprocess.run(['udevadm', 'trigger', '--subsystem-match=mem'], check=False)
print(f"[OK] udev rule written: {UDEV_RULE_FILE}")

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
print()
print("Setup complete. Reboot the robot to activate the changes.")
print("  - Thonny will open sandbox.py automatically at login.")
print("  - Double-click 'EMERGENCY_STOP' on the desktop to stop the robot.")
print()
print("To return to normal (web server only), delete the autostart file:")
print(f"  rm {THONNY_DESKTOP_FILE}")
