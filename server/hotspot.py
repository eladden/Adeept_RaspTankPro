#!/usr/bin/env python3
"""
hotspot.py - make sure the robot's Wi-Fi hotspot (Robot<X>) is up in sandbox mode.

On the older robots (Buster, no NetworkManager) the hotspot is started by
webServer.py itself (wifi_check -> create_ap). When startup.sh runs sandbox.py
instead of webServer.py, nothing starts it. sandbox.py calls ensure_hotspot()
to do the same thing webServer.py would have done.

Newer robots (NetworkManager) get their hotspot from wifi-hotspot-manager.service,
independent of what startup.sh runs, so there is nothing to do there.
"""

import os
import shutil
import socket
import subprocess

HOTSPOT_PASSWORD = '12345678'


def _robot_name():
    # Same file and fallback as webServer.read_robot_name()
    try:
        name_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'robot_name.txt')
        with open(name_file) as f:
            name = f.read().strip()
        if name:
            return name
    except Exception:
        pass
    return 'Robot1'


def _has_network():
    # Same test as webServer.wifi_check(): no route out means no Wi-Fi network was joined
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("1.1.1.1", 80))
        s.close()
        return True
    except OSError:
        return False


def _hotspot_running():
    return subprocess.run(['pgrep', '-f', 'create_ap'],
                          stdout=subprocess.DEVNULL).returncode == 0


def ensure_hotspot():
    """Start the create_ap hotspot if this robot uses it and it is not already up."""
    if shutil.which('create_ap') is None:
        return                      # NetworkManager robot: its own service handles it
    if _has_network():
        return                      # joined a known Wi-Fi network, no hotspot wanted
    if _hotspot_running():
        return                      # webServer.py already started it
    name = _robot_name()
    # start_new_session: the hotspot must outlive this program (Stop, emergency stop)
    subprocess.Popen(['sudo', 'create_ap', 'wlan0', 'eth0', name, HOTSPOT_PASSWORD],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    print('[hotspot] Starting hotspot %s (address 192.168.12.1)' % name)
