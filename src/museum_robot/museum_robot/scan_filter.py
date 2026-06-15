import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

RANGE_MIN = 0.20

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter')
        sub_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', pub_qos)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, sub_qos)
        self.get_logger().info('scan_filter ready: min_range=0.20m')

    def cb(self, msg):
        try:
            out = LaserScan()
            out.header = msg.header
            out.angle_min = msg.angle_min
            out.angle_max = msg.angle_max
            out.angle_increment = msg.angle_increment
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = RANGE_MIN
            out.range_max = msg.range_max
            out.ranges = [r if r >= RANGE_MIN else float('inf') for r in msg.ranges]
            out.intensities = msg.intensities
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'scan_filter cb error: {e}')

def main():
    rclpy.init()
    node = ScanFilterNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
