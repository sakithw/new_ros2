#!/usr/bin/env python3
"""PTY relay for RPLidar: holds the real serial port, presents a virtual port.

Problem: Every new process opening /dev/ttyUSB0 (CP210x USB serial) causes a
brief DTR glitch that trips the RPLidar's motor protection → health=2.
rplidar_composition cannot recover from this without a firmware RESET first,
but it doesn't send one.

Solution: This relay holds /dev/ttyUSB0 permanently (no more DTR glitches from
rplidar_composition). It presents a PTY slave at /tmp/lidar_pty which
rplidar_composition opens instead. PTY opens do NOT cause hardware DTR events.

Startup:
  1. Open real /dev/ttyUSB0 (first and only open of the real port)
  2. Send firmware RESET (0xA5 0x40) → health=0 within ~2s
  3. Create PTY pair, symlink slave as /tmp/lidar_pty
  4. Relay bytes bidirectionally: PTY master ↔ real serial port
  5. rplidar_composition opens /tmp/lidar_pty, gets health=0, starts scanning

Configure rplidar_composition: -p serial_port:=/tmp/lidar_pty
"""
import os, sys, pty, serial, threading, time, termios, tty, signal, select

REAL_PORT  = '/dev/lidar'
BAUD       = 1000000
PTY_LINK   = '/tmp/lidar_pty'

def cleanup(link):
    try:
        os.unlink(link)
    except Exception:
        pass

signal.signal(signal.SIGTERM, lambda *_: (cleanup(PTY_LINK), sys.exit(0)))
signal.signal(signal.SIGINT,  lambda *_: (cleanup(PTY_LINK), sys.exit(0)))

try:
    import serial as _serial_mod
except ImportError:
    sys.exit(0)

cleanup(PTY_LINK)

try:
    real = serial.Serial(REAL_PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f'[relay] failed to open {REAL_PORT}: {e}', flush=True)
    sys.exit(1)

# Disable HUPCL so close() doesn't drop DTR
import fcntl
attrs = termios.tcgetattr(real.fd)
attrs[2] &= ~termios.HUPCL
termios.tcsetattr(real.fd, termios.TCSANOW, attrs)
real.dtr = True

# Send RESET to clear health=2
real.write(bytes([0xA5, 0x40]))
real.flush()
print('[relay] RESET sent, waiting 3s for motor spinup...', flush=True)
time.sleep(3)
real.reset_input_buffer()
print('[relay] ready, creating PTY...', flush=True)

# Create PTY
master_fd, slave_fd = pty.openpty()
slave_path = os.ttyname(slave_fd)
os.symlink(slave_path, PTY_LINK)
print(f'[relay] PTY slave: {slave_path} → {PTY_LINK}', flush=True)

# Set master side to raw mode
tty.setraw(master_fd)

stop = threading.Event()

def pty_to_real():
    """Forward bytes from PTY master (rplidar writes) → real serial port."""
    while not stop.is_set():
        try:
            r, _, _ = select.select([master_fd], [], [], 0.5)
            if master_fd in r:
                data = os.read(master_fd, 4096)
                if data:
                    real.write(data)
                    real.flush()
        except Exception:
            pass

def real_to_pty():
    """Forward bytes from real serial port (lidar data) → PTY master."""
    while not stop.is_set():
        try:
            data = real.read(real.in_waiting or 1)
            if data:
                os.write(master_fd, data)
        except Exception:
            pass

t1 = threading.Thread(target=pty_to_real, daemon=True)
t2 = threading.Thread(target=real_to_pty, daemon=True)
t1.start()
t2.start()

print('[relay] forwarding data — rplidar_composition should use /tmp/lidar_pty', flush=True)
try:
    while True:
        time.sleep(1)
except (KeyboardInterrupt, SystemExit):
    pass
finally:
    stop.set()
    cleanup(PTY_LINK)
    real.close()
