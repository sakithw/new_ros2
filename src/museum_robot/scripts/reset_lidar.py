#!/usr/bin/env python3
"""Send firmware RESET to clear RPLidar health=2 error, then exit quickly.

health=2 is a sticky firmware error state. RESET (0xA5 0x40) clears it.
After RESET the motor spins up and health=0 within ~2s.

This script is run 3s before rplidar_composition starts (via a TimerAction
at t=27s while rplidar starts at t=30s). By keeping the gap under ~2s, the
motor is still spinning when rplidar_composition opens the port and checks health.

DO NOT run this at t=0: the 20+ second gap lets the motor stop and health=2
returns, defeating the purpose.
"""
import sys, time, termios
try:
    import serial
except ImportError:
    sys.exit(0)
try:
    with serial.Serial('/dev/lidar', 1000000, timeout=2) as s:
        attr = termios.tcgetattr(s.fd)
        attr[2] &= ~termios.HUPCL
        termios.tcsetattr(s.fd, termios.TCSANOW, attr)
        s.dtr = True
        s.write(bytes([0xA5, 0x40]))   # RESET — clears internal error state
        s.flush()
        time.sleep(3)                  # firmware reboots ~1s, health=0 by ~2s
except Exception:
    pass
