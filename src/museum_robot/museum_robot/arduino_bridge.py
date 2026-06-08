import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial, threading, math, time, termios

WHEEL_DIAMETER_M  = 0.12
TRACK_WIDTH_M     = 0.30
PULSES_PER_REV    = 20
WHEEL_CIRCUM_M    = math.pi * WHEEL_DIAMETER_M
TICKS_PER_M       = PULSES_PER_REV / WHEEL_CIRCUM_M
TF_HZ             = 50.0
ODOM_HZ           = 20.0
PORT              = '/dev/ttyAMA0'
BAUD              = 115200
WATCHDOG_S        = 2.0
STOP_LOCKOUT_S    = 0.20
REPEAT_HZ         = 5.0

# Short-distance step commands — keep robot in ramp phase, limit effective speed.
# Subsequent steps are re-sent by _drive_repeat_cb while button is held.
_STEP_CMD = {
    'FWD':   'F500.0',
    'BWD':   'B500.0',
    'LEFT':  'T-15.0',
    'RIGHT': 'T15.0',
}


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.tf_br    = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.prev_l = self.prev_r = None
        self._last_odom      = time.time()
        self._last_cmd_time  = time.time()
        self._current_state  = 'STOP'
        self._last_stop_sent = 0.0
        self._ser  = None
        self._lock = threading.Lock()

        self._publish_tf(self.get_clock().now(), 0.0, 0.0, 0.0)
        self.create_timer(1.0 / TF_HZ,     self._tf_timer_cb)
        self.create_timer(1.0 / ODOM_HZ,   self._odom_timer_cb)
        self.create_timer(WATCHDOG_S,       self._watchdog_cb)
        self.create_timer(1.0 / REPEAT_HZ, self._drive_repeat_cb)

        threading.Thread(target=self._serial_reader, daemon=True).start()
        self.get_logger().info('arduino_bridge ready.')

    # ── TF / Odom ─────────────────────────────────────────────────────────

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

    # ── Serial ────────────────────────────────────────────────────────────

    def _serial_reader(self):
        while rclpy.ok():
            try:
                with serial.Serial(PORT, BAUD, timeout=0.05,
                                   xonxoff=False, rtscts=False, dsrdtr=False) as ser:
                    # Disable HUPCL: keeps DTR asserted when port closes so the
                    # Arduino doesn't reset on every reconnect attempt.
                    # CLOCAL: ignore modem status lines (CD/RI) — avoids false HUP.
                    # IGNBRK: ignore UART break condition — avoids spurious EOF.
                    attr = termios.tcgetattr(ser.fd)
                    attr[2] |= termios.CLOCAL    # cflag
                    attr[2] &= ~termios.HUPCL    # cflag
                    attr[0] |= termios.IGNBRK    # iflag
                    termios.tcsetattr(ser.fd, termios.TCSANOW, attr)

                    self._ser = ser
                    ser.reset_input_buffer()
                    self.get_logger().info(f'Serial opened on {PORT} @ {BAUD}')
                    buf = b''
                    while rclpy.ok():
                        try:
                            n = ser.in_waiting
                            if n == 0:
                                time.sleep(0.005)
                                continue
                            chunk = ser.read(n)
                        except serial.SerialException as e:
                            if 'no data' in str(e):
                                # Spurious HUP — keep port open, retry
                                time.sleep(0.01)
                                continue
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
                            elif line not in ('DONE', 'STOPPED', 'READY'):
                                self.get_logger().info(f'Arduino: {line}')
            except Exception as e:
                self._ser = None
                self.get_logger().warn(f'Serial error: {e}, retrying in 2s')
                time.sleep(2)

    # ── Commands ──────────────────────────────────────────────────────────

    def _send(self, cmd):
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.write((cmd + '\n').encode())
            except Exception:
                pass

    def _cmd_cb(self, msg):
        self._last_cmd_time = time.time()
        lx = msg.linear.x
        az = msg.angular.z

        if abs(lx) < 0.01 and abs(az) < 0.01:
            if self._current_state != 'STOP':
                self._send('S')
                self._current_state = 'STOP'
                self._last_stop_sent = time.time()
                self.get_logger().info('Sent: S (STOP)')
            return

        new_state = ('FWD'   if lx >  0.01 else
                     'BWD'   if lx < -0.01 else
                     'LEFT'  if az >  0.01 else
                     'RIGHT')

        if new_state == self._current_state:
            return

        # Discard out-of-order motion commands arriving within 200ms of a STOP
        if self._current_state == 'STOP' and time.time() - self._last_stop_sent < STOP_LOCKOUT_S:
            return

        # Direction change: interrupt in-progress step
        if self._current_state != 'STOP':
            self._send('S')

        self._current_state = new_state
        cmd = _STEP_CMD[new_state]
        self._send(cmd)
        self.get_logger().info(f'Sent: {cmd} ({new_state})')

    def _drive_repeat_cb(self):
        """Repeat current drive step at REPEAT_HZ while button held."""
        state = self._current_state
        if state == 'STOP':
            return
        # Honour lockout: don't re-trigger within 200ms of a stop
        if time.time() - self._last_stop_sent < STOP_LOCKOUT_S:
            return
        self._send(_STEP_CMD[state])

    def _watchdog_cb(self):
        if time.time() - self._last_cmd_time > WATCHDOG_S:
            if self._current_state != 'STOP':
                self._send('S')
                self._current_state = 'STOP'
                self.get_logger().warn('Watchdog: cmd_vel lost — stopped')


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
