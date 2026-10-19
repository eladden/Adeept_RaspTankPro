#!/usr/bin/env python3
"""
emergency_stop.py - stop the student's program and force the motors off.

Run by the EMERGENCY STOP desktop shortcut (sudo python3 emergency_stop.py).

1. Ends the program started from sandbox.py. sandbox.py registers its process
   id in /tmp/sandbox.pid, so the exact process is found even when it runs
   inside Thonny (matching on the text "sandbox.py" would hit the Thonny window
   instead). It gets SIGTERM first, which runs sandbox.py's own cleanup, then
   SIGKILL if it is still alive after 2 seconds.
2. Sets every motor pin LOW from here, whatever happened in step 1. A program
   that is killed leaves its pins as they were, so a motor that was running
   would otherwise keep running.

sandbox.py imports register() / unregister() from this file.
"""

import os
import signal
import sys
import time

PID_FILE = '/tmp/sandbox.pid'


def _proc_stat(pid):
    """(state, start_time) of a process from /proc, or None if it is gone."""
    try:
        with open('/proc/%d/stat' % pid) as f:
            fields = f.read().rsplit(')', 1)[1].split()
        return fields[0], fields[19]      # fields 3 (state) and 22 (start time)
    except (OSError, IndexError, ValueError):
        return None


def register():
    """Called by sandbox.py: remember this process so the e-stop can find it."""
    try:
        stat = _proc_stat(os.getpid())
        with open(PID_FILE, 'w') as f:
            f.write('%d %s\n' % (os.getpid(), stat[1] if stat else ''))
    except OSError as e:
        print('[emergency_stop] could not write %s: %s' % (PID_FILE, e))


def unregister():
    """Called by sandbox.py on a normal exit."""
    try:
        with open(PID_FILE) as f:
            pid = int(f.read().split()[0])
        if pid == os.getpid():
            os.remove(PID_FILE)
    except (OSError, ValueError, IndexError):
        pass


def stop_student_program():
    try:
        with open(PID_FILE) as f:
            pid_text, start = f.read().split()
        pid = int(pid_text)
    except (OSError, ValueError):
        print('No sandbox program is registered (nothing to end).')
        return

    def alive():
        stat = _proc_stat(pid)
        return stat is not None and stat[1] == start and stat[0] != 'Z'

    if not alive():
        print('The registered program (pid %d) is no longer running.' % pid)
        return
    print('Ending the program (pid %d)...' % pid)
    os.kill(pid, signal.SIGTERM)
    for _ in range(20):
        time.sleep(0.1)
        if not alive():
            print('Program ended.')
            return
    os.kill(pid, signal.SIGKILL)
    print('Program did not end by itself, killed it.')


def force_motors_low():
    """Drive every motor pin LOW. No GPIO.cleanup(): that would let the pins float."""
    import RPi.GPIO as GPIO
    import move                           # pin numbers live in move.py
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in (move.Motor_A_EN, move.Motor_B_EN, move.Motor_A_Pin1,
                move.Motor_A_Pin2, move.Motor_B_Pin1, move.Motor_B_Pin2):
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    print('Motor pins set LOW.')


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        stop_student_program()
    finally:
        force_motors_low()
