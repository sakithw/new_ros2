#!/usr/bin/env python3
"""Spins up RPLiDAR motor and keeps it running after this script exits.

Clears HUPCL on the serial port so DTR stays asserted (motor spinning)
after the file descriptor is closed. rplidar_composition then connects
to an already-running motor and gets health=0 immediately.
No RESET command is sent — the motor spins up naturally from DTR assert.
"""
import sys, time, termios
try:
    import serial
except ImportError:
    sys.exit(0)
try:
    with serial.Serial('/dev/lidar', 1000000, timeout=2) as s:
        # Clear HUPCL: port close will not drop DTR → motor keeps spinning
        attr = termios.tcgetattr(s.fd)
        attr[2] &= ~termios.HUPCL
        termios.tcsetattr(s.fd, termios.TCSANOW, attr)
        s.dtr = True
        s.write(bytes([0xA5, 0x25]))   # STOP any active scan
        s.flush()
        time.sleep(7)                  # Wait for motor to reach operating speed
except Exception:
    pass
