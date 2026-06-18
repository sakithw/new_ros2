#!/usr/bin/env python3
"""Keep RPLidar motor spinning by holding /dev/lidar open with DTR asserted.

Linux serial ports allow multiple concurrent opens (no mandatory exclusive lock).
This process holds the fd open so DTR stays asserted even when rplidar_composition
crashes and closes its fd. Motor never stops → health never returns to 2 →
rplidar_composition always finds health=0 on respawn.

Startup sequence:
  1. Send RESET (0xA5 0x40) to clear any sticky health=2 error
  2. Wait 3s for firmware reboot and motor spin-up (health=0 by ~2s)
  3. Hold port open indefinitely, DTR=True — no more UART communication

rplidar_composition starts at t=5s (TimerAction) and opens the port concurrently.
Motor keeper's fd keeps DTR up; rplidar_composition's fd does all scan communication.
"""
import sys, time, termios, signal

try:
    import serial
except ImportError:
    sys.exit(0)

signal.signal(signal.SIGINT,  lambda *_: sys.exit(0))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

while True:
    try:
        with serial.Serial('/dev/lidar', 1000000, timeout=2) as s:
            attr = termios.tcgetattr(s.fd)
            attr[2] &= ~termios.HUPCL
            termios.tcsetattr(s.fd, termios.TCSANOW, attr)
            s.dtr = True
            s.write(bytes([0xA5, 0x40]))   # RESET — clears sticky health=2
            s.flush()
            time.sleep(3)                  # firmware reboots ~1s, health=0 by ~2s
            # Hold fd open indefinitely — no more UART writes
            while True:
                time.sleep(60)
    except Exception:
        time.sleep(1)
