#!/usr/bin/env python3
"""TCP relay for RPLidar: holds real serial port, bridges to localhost TCP.

Problem: opening /dev/lidar in any new process causes a DTR glitch → health=2.
Also, health=0 window after RESET is only ~1-2s — must start scanning in time.

Solution: This process opens /dev/lidar ONCE (no more DTR glitches ever).
rplidar_composition uses channel_type:=tcp → no serial port open → no DTR glitch.
On each new TCP connection (startup or respawn), relay sends RESET and waits 1.2s
(putting health=0) BEFORE forwarding any data. rplidar_composition's health check
hits the ~1-2s health=0 window and succeeds.

Configure rplidar_composition:
  -p channel_type:=tcp
  -p tcp_ip:=127.0.0.1
  -p tcp_port:=10660
"""
import socket, serial, threading, time, signal, sys, select

REAL_PORT  = '/dev/lidar'
BAUD       = 1000000
TCP_HOST   = '127.0.0.1'
TCP_PORT   = 10660
RESET_WAIT = 1.2   # seconds — health=0 window opens ~1s after RESET

def main():
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    signal.signal(signal.SIGINT,  lambda *_: sys.exit(0))

    try:
        import serial as _
    except ImportError:
        sys.exit(0)

    try:
        real = serial.Serial(REAL_PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f'[tcp-relay] cannot open {REAL_PORT}: {e}', flush=True)
        sys.exit(1)

    real.dtr = True

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_HOST, TCP_PORT))
    srv.listen(1)
    print(f'[tcp-relay] listening on {TCP_HOST}:{TCP_PORT}', flush=True)

    def do_reset():
        """Send RESET and wait for health=0 window before relaying."""
        real.reset_input_buffer()
        real.write(bytes([0xA5, 0x40]))   # RESET — clears sticky health=2
        real.flush()
        print('[tcp-relay] RESET sent, waiting for health=0 window...', flush=True)
        time.sleep(RESET_WAIT)
        real.reset_input_buffer()
        print('[tcp-relay] ready to relay', flush=True)

    # Initial RESET so first connection finds health=0 immediately
    do_reset()

    while True:
        conn, addr = srv.accept()
        print(f'[tcp-relay] connected {addr}', flush=True)
        stop = threading.Event()

        def tcp_to_serial():
            while not stop.is_set():
                try:
                    r, _, _ = select.select([conn], [], [], 0.5)
                    if conn in r:
                        data = conn.recv(4096)
                        if not data:
                            stop.set()
                            break
                        real.write(data)
                        real.flush()
                except Exception:
                    stop.set()

        def serial_to_tcp():
            while not stop.is_set():
                try:
                    data = real.read(real.in_waiting or 1)
                    if data:
                        conn.sendall(data)
                except Exception:
                    stop.set()

        t1 = threading.Thread(target=tcp_to_serial, daemon=True)
        t2 = threading.Thread(target=serial_to_tcp, daemon=True)
        t1.start(); t2.start()
        t1.join();  t2.join()

        conn.close()
        print('[tcp-relay] disconnected — sending RESET for next connection', flush=True)
        # RESET before accepting next connection so health=0 when rplidar reconnects
        do_reset()

if __name__ == '__main__':
    main()
