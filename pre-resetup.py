#!/usr/bin/env python3
"""
pre-resetup.py — Prepare the robot for a clean re-run of setup.py

Run this after re-cloning the repo and before setup.py, whenever the
systemd services may still be active from a previous installation.

Usage:
    sudo python3 pre-resetup.py
"""

import subprocess

for svc in ('Adeept_Robot.service', 'wifi-hotspot-manager.service'):
    subprocess.run(['sudo', 'systemctl', 'stop', svc], capture_output=True)
    print(f'[OK] Stopped {svc}')

print()
print('Ready. Run next: sudo python3 setup.py')
