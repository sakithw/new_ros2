import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess, time, serial

LIDAR_PORT  = '/dev/lidar'
LIDAR_BAUD  = 1000000
SCAN_TIMEOUT_S  = 8.0
RESET_HOLD_S    = 10.0   # hold port open after RESET so rplidar can't reconnect too early

class LidarWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_watchdog')
        self._last_scan  = time.time()
        self._resetting  = False
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_timer(2.0, self._check_cb)
        self.get_logger().info('lidar_watchdog ready.')

    def _scan_cb(self, _msg):
        self._last_scan = time.time()

    def _check_cb(self):
        if self._resetting:
            return
        age = time.time() - self._last_scan
        if age > SCAN_TIMEOUT_S:
            self.get_logger().warn(
                f'No /scan for {age:.1f}s — resetting LiDAR and killing rplidar_composition')
            self._resetting = True
            # Kill rplidar first so it releases the serial port
            subprocess.run(['pkill', '-SIGINT', '-f', 'rplidar_composition'], check=False)
            time.sleep(1.0)
            # Send hardware RESET and hold the port to enforce the respawn delay
            try:
                with serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=2) as s:
                    s.write(bytes([0xA5, 0x40]))  # RESET
                    s.flush()
                    self.get_logger().info(f'LiDAR RESET sent, holding port for {RESET_HOLD_S}s')
                    time.sleep(RESET_HOLD_S)
            except Exception as e:
                self.get_logger().warn(f'LiDAR reset error: {e}')
                time.sleep(RESET_HOLD_S)
            self._last_scan = time.time()
            self._resetting = False
            self.get_logger().info('LiDAR reset done — rplidar_composition will respawn shortly')

def main():
    rclpy.init()
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
