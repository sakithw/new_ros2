import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial, threading, math, time, re

ODOM_RE = re.compile(r'(-?\d+),(-?\d+),([-\d.]+)')

WHEEL_DIAMETER_M = 0.12
TRACK_WIDTH_M    = 0.30
PULSES_PER_REV   = 20
WHEEL_CIRCUM_M   = math.pi * WHEEL_DIAMETER_M
TICKS_PER_M      = PULSES_PER_REV / WHEEL_CIRCUM_M
TF_HZ            = 50.0
ODOM_HZ          = 20.0
WATCHDOG_S       = 2.0
PORT             = '/dev/ttyAMA0'
BAUD             = 115200

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.tf_br    = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.x = self.y = self.yaw = 0.0
        self.prev_l = self.prev_r = None
        self._last_cmd = time.time()
        self._ser = None
        self._lock = threading.Lock()
        self._publish_tf(self.get_clock().now(), 0.0, 0.0, 0.0)
        self.create_timer(1.0 / TF_HZ,   self._tf_timer_cb)
        self.create_timer(1.0 / ODOM_HZ, self._odom_timer_cb)
        # self.create_timer(WATCHDOG_S, self._watchdog_cb)  # disabled: collides with ODOM stream
        threading.Thread(target=self._serial_reader, daemon=True).start()
        self.get_logger().info('arduino_bridge ready.')

    def _publish_tf(self, stamp, x, y, yaw):
        ts = TransformStamped()
        ts.header.stamp    = stamp.to_msg()
        ts.header.frame_id = 'odom'
        ts.child_frame_id  = 'base_link'
        ts.transform.translation.x = x
        ts.transform.translation.y = y
        ts.transform.translation.z = 0.0
        ts.transform.rotation.z = math.sin(yaw / 2)
        ts.transform.rotation.w = math.cos(yaw / 2)
        self.tf_br.sendTransform(ts)

    def _tf_timer_cb(self):
        self._publish_tf(self.get_clock().now(), self.x, self.y, self.yaw)

    def _odom_timer_cb(self):
        now  = self.get_clock().now()
        msg  = Odometry()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        msg.pose.covariance[0]  = 0.1
        msg.pose.covariance[7]  = 0.1
        msg.pose.covariance[35] = 0.2
        msg.twist.covariance[0]  = 0.1
        msg.twist.covariance[35] = 0.2
        self.odom_pub.publish(msg)

    def _handle_odom(self, left_ticks, right_ticks, yaw_deg):
        yaw = math.radians(yaw_deg)
        if self.prev_l is None:
            self.prev_l = left_ticks
            self.prev_r = right_ticks
            self.yaw    = yaw
            self._publish_tf(self.get_clock().now(), self.x, self.y, self.yaw)
            return
        dl = (left_ticks  - self.prev_l) / TICKS_PER_M
        dr = (right_ticks - self.prev_r) / TICKS_PER_M
        self.prev_l = left_ticks
        self.prev_r = right_ticks
        self.yaw    = yaw
        dc = (dl + dr) / 2.0
        self.x += dc * math.cos(self.yaw)
        self.y += dc * math.sin(self.yaw)
        self._publish_tf(self.get_clock().now(), self.x, self.y, self.yaw)

    def _serial_reader(self):
        while rclpy.ok():
            try:
                with serial.Serial(PORT, BAUD, timeout=0.1,
                                   xonxoff=False, rtscts=False, dsrdtr=False) as ser:
                    self._ser = ser
                    ser.reset_input_buffer()
                    self.get_logger().info(f'Serial opened on {PORT} @ {BAUD}')
                    buf = b''
                    while rclpy.ok():
                        try:
                            chunk = ser.read(ser.in_waiting or 64)
                        except serial.SerialException:
                            ser.reset_input_buffer()
                            buf = b''
                            continue
                        if chunk:
                            buf += chunk
                        if len(buf) > 4096:
                            buf = buf[-2048:]
                        while b'\n' in buf:
                            raw, buf = buf.split(b'\n', 1)
                            line = raw.decode('utf-8', errors='ignore').strip()
                            if not line:
                                continue
                            # Use regex to extract int,int,float payload even when
                            # leading "ODOM:" prefix bytes are dropped by UART noise
                            m = ODOM_RE.search(line)
                            if m:
                                try:
                                    self._handle_odom(
                                        int(m.group(1)),
                                        int(m.group(2)),
                                        float(m.group(3)))
                                except ValueError:
                                    pass
                            else:
                                self.get_logger().info(f'Arduino: {line}')
            except Exception as e:
                self._ser = None
                self.get_logger().warn(f'Serial error: {e}, retrying in 2s')
                time.sleep(2)

    def _send(self, cmd):
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.write((cmd + '\n').encode())
                    time.sleep(0.05)
            except Exception:
                pass

    def _cmd_cb(self, msg):
        self._last_cmd = time.time()
        lx = msg.linear.x
        az = msg.angular.z

        # Determine desired movement state
        if abs(lx) < 0.01 and abs(az) < 0.01:
            cmd_str = 'S\n'
            state = 'STOP'
        elif lx > 0.01:
            cmd_str = 'F1000.0\n'  # Large distance, interrupted by S
            state = 'FWD'
        elif lx < -0.01:
            cmd_str = 'B1000.0\n'
            state = 'BWD'
        elif az > 0.01:
            cmd_str = 'T360.0\n'   # Large angle, interrupted by S
            state = 'LEFT'
        elif az < -0.01:
            cmd_str = 'T-360.0\n'
            state = 'RIGHT'
        else:
            cmd_str = 'S\n'
            state = 'STOP'

        # Initialize state tracking if not present
        if not hasattr(self, '_current_state'):
            self._current_state = 'UNKNOWN'

        # Only send command if the state has changed to prevent buffer spam
        if state != self._current_state:
            self._send(cmd_str)
            self.get_logger().info(f'Sent to Arduino: {cmd_str.strip()}')
            self._current_state = state

    def _watchdog_cb(self):
        if time.time() - self._last_cmd > WATCHDOG_S:
            self._send('S')

def main():
    rclpy.init()
    node = ArduinoBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
