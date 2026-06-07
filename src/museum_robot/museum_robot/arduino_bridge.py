import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf2_ros import TransformBroadcaster
import serial, threading, math, time

WHEEL_DIAMETER_M  = 0.12
TRACK_WIDTH_M     = 0.30
PULSES_PER_REV    = 20
WHEEL_CIRCUM_M    = math.pi * WHEEL_DIAMETER_M
TICKS_PER_M       = PULSES_PER_REV / WHEEL_CIRCUM_M
TF_HZ             = 50.0
ODOM_HZ           = 20.0
DRIVE_HZ          = 10.0
PORT              = '/dev/ttyAMA0'
BAUD              = 115200
MAX_SPEED_MS      = 0.20   # cap forward/back speed for mapping
US_OBSTACLE_CM    = 25.0   # auto-stop if front ultrasonic reads closer than this


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.tf_br    = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.us_pub   = self.create_publisher(Range, '/ultrasonic', 10)
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.prev_l = self.prev_r = None
        self._last_odom     = time.time()
        self._current_state = 'STOP'
        self._lx            = 0.0
        self._us_front_cm   = 999.0   # last ultrasonic reading (default: clear)
        self._ser  = None
        self._lock = threading.Lock()

        self._publish_tf(self.get_clock().now(), 0.0, 0.0, 0.0)
        self.create_timer(1.0 / TF_HZ,    self._tf_timer_cb)
        self.create_timer(1.0 / ODOM_HZ,  self._odom_timer_cb)
        self.create_timer(1.0 / DRIVE_HZ, self._drive_timer_cb)
        # NO watchdog timer — stop button owns stopping

        threading.Thread(target=self._serial_reader, daemon=True).start()
        self.get_logger().info('arduino_bridge ready.')

    # ── TF / Odom ────────────────────────────────────────────────────────────

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
        now = self.get_clock().now()
        msg = Odometry()
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
        if abs(left_ticks) >= 1000000 or abs(right_ticks) >= 1000000:
            return
        if yaw_deg < -360.0 or yaw_deg > 360.0:
            return
        self._last_odom = time.time()
        yaw = math.radians(yaw_deg)
        if self.prev_l is None:
            self.prev_l, self.prev_r, self.yaw = left_ticks, right_ticks, yaw
            self._publish_tf(self.get_clock().now(), self.x, self.y, self.yaw)
            return
        dl = (left_ticks  - self.prev_l) / TICKS_PER_M
        dr = (right_ticks - self.prev_r) / TICKS_PER_M
        self.prev_l, self.prev_r, self.yaw = left_ticks, right_ticks, yaw
        dc = (dl + dr) / 2.0
        self.x += dc * math.cos(self.yaw)
        self.y += dc * math.sin(self.yaw)
        self._publish_tf(self.get_clock().now(), self.x, self.y, self.yaw)

    # ── Serial ───────────────────────────────────────────────────────────────

    def _serial_reader(self):
        while rclpy.ok():
            try:
                with serial.Serial(PORT, BAUD, timeout=0.05,
                                   xonxoff=False, rtscts=False, dsrdtr=False) as ser:
                    self._ser = ser
                    ser.reset_input_buffer()
                    self.get_logger().info(f'Serial opened on {PORT} @ {BAUD}')
                    buf = b''
                    while rclpy.ok():
                        try:
                            chunk = ser.read(ser.in_waiting or 64)
                        except serial.SerialException as e:
                            self.get_logger().warn(f'Serial read error: {e}')
                            buf = b''
                            time.sleep(1.0)
                            break
                        if chunk:
                            buf += chunk
                        if len(buf) > 4096:
                            buf = buf[-2048:]
                        while b'\n' in buf:
                            raw, buf = buf.split(b'\n', 1)
                            line = raw.decode('utf-8', errors='ignore').strip()
                            if not line:
                                continue
                            if line[:5] == 'ODOM:':
                                parts = line[5:].split(',')
                                if len(parts) == 3:
                                    try:
                                        self._handle_odom(
                                            int(parts[0]),
                                            int(parts[1]),
                                            float(parts[2]))
                                    except ValueError:
                                        pass
                            elif line[:3] == 'US:':
                                self._handle_ultrasonic(line[3:])
                            elif line not in ('DONE', 'STOPPED', 'READY'):
                                # Log unknowns (not routine status) to find new formats
                                self.get_logger().info(f'Arduino raw: {line}')
            except Exception as e:
                self._ser = None
                self.get_logger().warn(f'Serial error: {e}, retrying in 2s')
                time.sleep(2)

    def _handle_ultrasonic(self, payload: str):
        # Expects "US:<front_cm>" or "US:<front_cm>,<rear_cm>"
        try:
            parts = payload.split(',')
            front_cm = float(parts[0])
            self._us_front_cm = front_cm

            msg = Range()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.radiation_type  = Range.ULTRASOUND
            msg.field_of_view   = 0.26   # ~15 degrees
            msg.min_range       = 0.02
            msg.max_range       = 4.00
            msg.range           = front_cm / 100.0  # cm → m
            self.us_pub.publish(msg)
        except (ValueError, IndexError):
            pass

    # ── Commands ─────────────────────────────────────────────────────────────

    def _send(self, cmd):
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.write((cmd + '\n').encode())
            except Exception:
                pass

    def _drive_timer_cb(self):
        if self._current_state == 'FWD':
            # Obstacle check — ultrasonic front
            if self._us_front_cm < US_OBSTACLE_CM:
                self._send('S')
                self._current_state = 'STOP'
                self.get_logger().warn(
                    f'Obstacle {self._us_front_cm:.0f}cm ahead — auto-stopped')
                return
            speed = min(abs(self._lx), MAX_SPEED_MS)
            dist  = round(speed * (1.0 / DRIVE_HZ) * 100.0, 1)
            if dist > 0:
                self._send(f'F{dist}')
        elif self._current_state == 'BWD':
            speed = min(abs(self._lx), MAX_SPEED_MS)
            dist  = round(speed * (1.0 / DRIVE_HZ) * 100.0, 1)
            if dist > 0:
                self._send(f'B{dist}')

    def _cmd_cb(self, msg):
        lx = msg.linear.x
        az = msg.angular.z
        self._lx = lx

        if abs(lx) < 0.01 and abs(az) < 0.01:
            # Flask publishes {0,0} continuously when idle — only act on transition
            if self._current_state != 'STOP':
                self._send('S')
                self._current_state = 'STOP'
                self.get_logger().info('Arduino: S (STOP)')
            return

        if abs(lx) >= 0.01:
            new_state = 'FWD' if lx > 0 else 'BWD'
        else:
            new_state = 'LEFT' if az > 0 else 'RIGHT'

        if new_state == self._current_state:
            return

        self._current_state = new_state
        if new_state == 'LEFT':
            self._send('T10.0')
            self.get_logger().info('Arduino: T10.0 (LEFT)')
        elif new_state == 'RIGHT':
            self._send('T-10.0')
            self.get_logger().info('Arduino: T-10.0 (RIGHT)')
        # FWD/BWD: _drive_timer_cb sends incremental F/B


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
