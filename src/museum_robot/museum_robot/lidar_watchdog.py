import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess, time

SCAN_TIMEOUT_S   = 30.0   # seconds of no /scan before acting
# rplidar starts via 5s TimerAction; give it extra time before watchdog can fire.
_STARTUP_GRACE_S = 20.0


class LidarWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_watchdog')
        self._last_scan = time.time() + _STARTUP_GRACE_S
        self._acting    = False
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_timer(5.0, self._check_cb)
        self.get_logger().info('lidar_watchdog ready.')

    def _scan_cb(self, _msg):
        self._last_scan = time.time()

    def _rplidar_running(self) -> bool:
        return subprocess.run(['pgrep', '-f', 'rplidar_composition'],
                              capture_output=True).returncode == 0

    def _check_cb(self):
        if self._acting:
            return
        age = time.time() - self._last_scan
        if age <= SCAN_TIMEOUT_S:
            return

        self._acting = True
        self.get_logger().warn(f'No /scan for {age:.1f}s')

        if self._rplidar_running():
            self.get_logger().warn('Soft-killing stuck rplidar_composition')
            subprocess.run(['pkill', '-SIGINT', '-f', 'rplidar_composition'], check=False)
            for _ in range(5):
                time.sleep(1.0)
                if not self._rplidar_running():
                    break

        # Motor keeper keeps DTR asserted and motor spinning continuously,
        # so rplidar_composition will find health=0 when it respawns.
        self.get_logger().info(
            'rplidar_composition killed — motor keeper will keep motor running '
            'until respawn (5s delay)')

        self._last_scan = time.time()
        self._acting    = False


def main():
    rclpy.init()
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
