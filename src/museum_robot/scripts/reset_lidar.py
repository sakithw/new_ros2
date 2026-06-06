#!/usr/bin/env python3
"""Warms up RPLiDAR motor. Takes 22 seconds intentionally."""
import sys, time
try:
    import serial
except ImportError:
    sys.exit(0)
try:
    with serial.Serial('/dev/lidar', 1000000, timeout=2) as s:
        s.write(bytes([0xA5, 0x40]))
        s.flush()
        time.sleep(22)
except Exception:
    pass
