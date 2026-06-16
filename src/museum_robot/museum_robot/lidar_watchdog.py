import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess, time, serial, termios

LIDAR_PORT       = '/dev/lidar'
LIDAR_BAUD       = 1000000
SCAN_TIMEOUT_S   = 30.0   # seconds of no /scan before acting
RESET_HOLD_S     = 10.0   # hold port after hw RESET
RESET_COOLDOWN_S = 60.0   # minimum seconds between hw RESETs
HEALTH_FAIL_MAX  = 3      # consecutive bad health checks before hw RESET
# rplidar starts via 24s TimerAction; give it extra time before watchdog can fire.
_STARTUP_GRACE_S = 40.0


class LidarWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_watchdog')
        self._last_scan    = time.time() + _STARTUP_GRACE_S
        self._last_reset   = 0.0
        self._health_fails = 0
        self._acting       = False
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_timer(5.0, self._check_cb)
        self.get_logger().info('lidar_watchdog ready.')

    def _scan_cb(self, _msg):
        self._last_scan    = time.time()
        self._health_fails = 0   # scans flowing — clear failure count

    def _rplidar_running(self) -> bool:
        return subprocess.run(['pgrep', '-f', 'rplidar_composition'],
                              capture_output=True).returncode == 0

    def _soft_kill(self):
        subprocess.run(['pkill', '-SIGINT', '-f', 'rplidar_composition'], check=False)

    def _clear_hupcl(self, fd: int):
        """Clear HUPCL so DTR stays asserted after port closes → motor keeps spinning."""
        attr = termios.tcgetattr(fd)
        attr[2] &= ~termios.HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, attr)

    def _check_health(self) -> int:
        """Returns health status byte: 0=OK, 1=warn, 2=error, 255=comm failure.

        Asserts DTR (motor power) and clears HUPCL so the motor keeps spinning
        after this method returns, giving rplidar_composition a warm motor.
        Waits 5s for the motor to reach operating speed before querying health.
        """
        try:
            with serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=2) as s:
                self._clear_hupcl(s.fd)   # motor stays on after port releases
                s.dtr = True
                s.write(bytes([0xA5, 0x25]))   # STOP any active scan
                s.flush()
                time.sleep(5.0)               # wait for motor to reach operating speed
                s.reset_input_buffer()
                s.write(bytes([0xA5, 0x52]))   # GET_HEALTH
                s.flush()
                time.sleep(0.5)
                raw = s.read(s.in_waiting)
                for i in range(len(raw) - 6):
                    if raw[i] == 0xA5 and raw[i + 1] == 0x5A:
                        if i + 10 <= len(raw):
                            return raw[i + 7]
        except Exception as e:
            self.get_logger().warn(f'Health check serial error: {e}')
        return 255

    def _hw_reset(self):
        try:
            with serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=2) as s:
                self._clear_hupcl(s.fd)   # motor stays on after port releases
                s.dtr = True
                s.write(bytes([0xA5, 0x40]))   # RESET
                s.flush()
                self.get_logger().info(
                    f'LiDAR hw RESET sent, holding port for {RESET_HOLD_S}s')
                time.sleep(RESET_HOLD_S)   # firmware reboots ~1s, motor spins up ~5s
        except Exception as e:
            self.get_logger().warn(f'LiDAR hw reset error: {e}')
            time.sleep(RESET_HOLD_S)

    def _check_cb(self):
        if self._acting:
            return
        age = time.time() - self._last_scan
        if age <= SCAN_TIMEOUT_S:
            return

        self._acting = True

        # ── Step 1: soft-kill stuck rplidar so we can access the port ─────
        if self._rplidar_running():
            self.get_logger().warn(
                f'No /scan for {age:.1f}s — soft-killing stuck rplidar_composition')
            self._soft_kill()
            # Wait up to 5 s for process to release the port
            for _ in range(5):
                time.sleep(1.0)
                if not self._rplidar_running():
                    break

        # ── Step 2: health check (port is now free) ───────────────────────
        status = self._check_health()
        self.get_logger().info(f'Post-kill health check: status={status}')

        if status == 0:
            # Hardware is healthy — a soft respawn will sort it out
            self._health_fails = 0
            self.get_logger().info(
                'LiDAR health OK — letting bringup respawn naturally (no hw RESET)')
            # Give the respawn window space; don't re-fire for another full cycle
            self._last_scan = time.time()
            self._acting    = False
            return

        # ── Step 3: bad health — track consecutive failures ───────────────
        self._health_fails += 1
        self.get_logger().warn(
            f'LiDAR health={status} — failure {self._health_fails}/{HEALTH_FAIL_MAX}')

        if self._health_fails < HEALTH_FAIL_MAX:
            self.get_logger().info(
                f'Waiting for {HEALTH_FAIL_MAX - self._health_fails} more failure(s) before hw RESET')
            self._last_scan = time.time()
            self._acting    = False
            return

        # ── Step 4: 3 consecutive bad health — hw RESET if cooldown allows ─
        cooldown_remaining = RESET_COOLDOWN_S - (time.time() - self._last_reset)
        if cooldown_remaining > 0:
            self.get_logger().warn(
                f'Health failed {self._health_fails}x but cooldown active '
                f'({cooldown_remaining:.0f}s remaining) — skipping hw RESET')
            self._last_scan = time.time()
            self._acting    = False
            return

        self._health_fails = 0
        self.get_logger().warn(
            f'Escalating to hw RESET after {HEALTH_FAIL_MAX} consecutive health failures')
        self._hw_reset()
        self._last_reset = time.time()
        self._last_scan  = time.time()
        self._acting     = False
        self.get_logger().info('hw RESET done — rplidar_composition will respawn shortly')


def main():
    rclpy.init()
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
