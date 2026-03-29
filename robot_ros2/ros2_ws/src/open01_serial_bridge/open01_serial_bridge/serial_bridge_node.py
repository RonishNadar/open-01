#!/usr/bin/env python3
"""
OPEN-01 Serial Bridge Node
Bridges ESP32 binary UART protocol <-> ROS2 topics.

Packet format:
  [0xAA][0x55][VERSION][MSG_TYPE][PAYLOAD_LEN][...PAYLOAD...][CRC_HI][CRC_LO][0xFF]
  CRC16-CCITT (init=0xFFFF, poly=0x1021) over [VERSION, MSG_TYPE, PAYLOAD_LEN, ...PAYLOAD]

RX thread feeds a ring buffer; state machine parses packets.
ROS2 spin thread handles all publishing + cmd_vel TX.
"""

import struct
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, BatteryState, Range, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

import math

# ─── Protocol constants ────────────────────────────────────────────────────────
FRAME_START_1   = 0xAA
FRAME_START_2   = 0x55
FRAME_END       = 0xFF
PROTOCOL_VER    = 0x01

MSG_CMD_VELOCITY = 0x10
MSG_CMD_ESTOP    = 0x11
MSG_CMD_PING     = 0x12
MSG_TELEMETRY    = 0x20
MSG_PONG         = 0x21

SUB_IMU       = 0x01   # 24B: 6x float (ax, ay, az, gx, gy, gz)
SUB_TOF       = 0x02   # 6B:  3x uint16 (left, back, right) mm
SUB_ODOM      = 0x03   # 24B: 6x float (x, y, theta, linear, angular, unused)
SUB_BATTERY   = 0x04   # 4B:  float voltage
SUB_TIMESTAMP = 0x05   # 4B:  uint32 ms

# ─── CRC16-CCITT ───────────────────────────────────────────────────────────────

def crc16_ccitt(data: bytes, init: int = 0x0000) -> int:
    """CRC16-CCITT (poly=0x1021). Matches ESP32 firmware implementation."""
    crc = init
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ─── Packet builder ────────────────────────────────────────────────────────────

def build_packet(msg_type: int, payload: bytes) -> bytes:
    """Construct a framed packet with CRC."""
    version = PROTOCOL_VER
    payload_len = len(payload)
    crc_data = bytes([version, msg_type, payload_len]) + payload
    crc = crc16_ccitt(crc_data)
    return bytes([
        FRAME_START_1, FRAME_START_2,
        version, msg_type, payload_len,
        *payload,
        (crc >> 8) & 0xFF, crc & 0xFF,
        FRAME_END
    ])

# ─── Parser state machine ──────────────────────────────────────────────────────

class PacketParser:
    """
    State machine that pulls bytes one at a time and emits complete packets.
    States: WAIT_START1 → WAIT_START2 → VERSION → MSG_TYPE → PAYLOAD_LEN
            → PAYLOAD → CRC_HI → CRC_LO → END
    """

    WAIT_START1  = 0
    WAIT_START2  = 1
    VERSION      = 2
    MSG_TYPE     = 3
    PAYLOAD_LEN  = 4
    PAYLOAD      = 5
    CRC_HI       = 6
    CRC_LO       = 7
    END          = 8

    def __init__(self):
        self._state       = self.WAIT_START1
        self._version     = 0
        self._msg_type    = 0
        self._payload_len = 0
        self._payload     = bytearray()
        self._crc_hi      = 0
        self._packets     = deque()
        self.bytes_rx     = 0
        self.packets_ok   = 0
        self.crc_errors   = 0
        self.frame_errors = 0

    def feed(self, data: bytes):
        for byte in data:
            self.bytes_rx += 1
            self._process_byte(byte)

    def _reset(self):
        self._state   = self.WAIT_START1
        self._payload = bytearray()

    def _process_byte(self, b: int):
        s = self._state

        if s == self.WAIT_START1:
            if b == FRAME_START_1:
                self._state = self.WAIT_START2

        elif s == self.WAIT_START2:
            if b == FRAME_START_2:
                self._state = self.VERSION
            else:
                self.frame_errors += 1
                self._reset()

        elif s == self.VERSION:
            self._version = b
            self._state = self.MSG_TYPE

        elif s == self.MSG_TYPE:
            self._msg_type = b
            self._state = self.PAYLOAD_LEN

        elif s == self.PAYLOAD_LEN:
            self._payload_len = b
            self._payload = bytearray()
            if b == 0:
                self._state = self.CRC_HI
            else:
                self._state = self.PAYLOAD

        elif s == self.PAYLOAD:
            self._payload.append(b)
            if len(self._payload) == self._payload_len:
                self._state = self.CRC_HI

        elif s == self.CRC_HI:
            self._crc_hi = b
            self._state = self.CRC_LO

        elif s == self.CRC_LO:
            received_crc = (self._crc_hi << 8) | b
            crc_input = bytes([self._version, self._msg_type, self._payload_len]) \
                        + bytes(self._payload)
            expected_crc = crc16_ccitt(crc_input)
            if received_crc == expected_crc:
                self._state = self.END
            else:
                self.crc_errors += 1
                self._reset()

        elif s == self.END:
            if b == FRAME_END:
                self._packets.append((self._msg_type, bytes(self._payload)))
                self.packets_ok += 1
            else:
                self.frame_errors += 1
            self._reset()

    def pop_packet(self):
        return self._packets.popleft() if self._packets else None

    def has_packet(self) -> bool:
        return len(self._packets) > 0

# ─── Main node ─────────────────────────────────────────────────────────────────

class SerialBridgeNode(Node):

    # Wheel geometry from URDF
    WHEEL_BASE   = 0.2065    # y offset = 0.10325 * 2
    WHEEL_RADIUS = 0.03432   # z offset from base_link to wheel center

    def __init__(self):
        super().__init__('serial_bridge')

        # ── Parameters ──
        self.declare_parameter('port',          '/dev/ttyS0')
        self.declare_parameter('baud',          460800)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('imu_frame_id',  'imu_link')
        self.declare_parameter('publish_tf',    True)

        port             = self.get_parameter('port').value
        baud             = self.get_parameter('baud').value
        self._base_frame = self.get_parameter('base_frame_id').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._imu_frame  = self.get_parameter('imu_frame_id').value
        self._publish_tf = self.get_parameter('publish_tf').value

        # ── Serial port ──
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
            )
            self.get_logger().info(f'Opened serial port {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port {port}: {e}')
            raise

        self._parser      = PacketParser()
        self._serial_lock = threading.Lock()

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ──
        self._pub_odom         = self.create_publisher(Odometry,     '/odom',          sensor_qos)
        self._pub_imu          = self.create_publisher(Imu,          '/imu/data',      sensor_qos)
        self._pub_battery      = self.create_publisher(BatteryState, '/battery_state', sensor_qos)
        self._pub_tof_l        = self.create_publisher(Range,        '/tof/left',      sensor_qos)
        self._pub_tof_b        = self.create_publisher(Range,        '/tof/back',      sensor_qos)
        self._pub_tof_r        = self.create_publisher(Range,        '/tof/right',     sensor_qos)
        self._pub_joint_states = self.create_publisher(JointState,   '/joint_states',  sensor_qos)

        # ── TF broadcaster ──
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # ── Subscriber ──
        self._sub_cmdvel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ── Timers ──
        self._rx_timer   = self.create_timer(0.01, self._rx_timer_cb)
        self._diag_timer = self.create_timer(5.0,  self._diag_cb)

        # ── RX thread ──
        self._rx_buf      = deque()
        self._rx_buf_lock = threading.Lock()
        self._running     = True
        self._rx_thread   = threading.Thread(target=self._rx_thread_fn, daemon=True)
        self._rx_thread.start()

        # ── Wheel position accumulators ──
        self._wheel_left_pos  = 0.0
        self._wheel_right_pos = 0.0
        self._last_odom_time  = None

        self.get_logger().info('serial_bridge node started')

    # ── RX thread ────────────────────────────────────────────────────────────

    def _rx_thread_fn(self):
        while self._running:
            try:
                data = self._serial.read(256)
                if data:
                    with self._rx_buf_lock:
                        self._rx_buf.append(data)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    # ── RX timer ─────────────────────────────────────────────────────────────

    def _rx_timer_cb(self):
        chunks = []
        with self._rx_buf_lock:
            while self._rx_buf:
                chunks.append(self._rx_buf.popleft())

        for chunk in chunks:
            self._parser.feed(chunk)

        while self._parser.has_packet():
            msg_type, payload = self._parser.pop_packet()
            self._dispatch_packet(msg_type, payload)

    def _dispatch_packet(self, msg_type: int, payload: bytes):
        if msg_type == MSG_TELEMETRY:
            self._parse_telemetry(payload)
        elif msg_type == MSG_PONG:
            self.get_logger().debug('PONG received')
        else:
            self.get_logger().warn(f'Unknown msg_type: 0x{msg_type:02X}')

    # ── Telemetry parser ─────────────────────────────────────────────────────

    def _parse_telemetry(self, payload: bytes):
        now = self.get_clock().now().to_msg()
        idx = 0
        imu_data = odom_data = tof_data = battery_data = ts_data = None

        while idx + 2 <= len(payload):
            sub_id  = payload[idx]
            sub_len = payload[idx + 1]
            idx += 2

            if idx + sub_len > len(payload):
                self.get_logger().warn('Telemetry sub-block truncated')
                break

            sub_payload = payload[idx:idx + sub_len]
            idx += sub_len

            if sub_id == SUB_IMU and sub_len == 24:
                imu_data = struct.unpack_from('<6f', sub_payload)
            elif sub_id == SUB_TOF and sub_len == 6:
                tof_data = struct.unpack_from('<3H', sub_payload)
            elif sub_id == SUB_ODOM and sub_len == 24:
                odom_data = struct.unpack_from('<6f', sub_payload)
            elif sub_id == SUB_BATTERY and sub_len == 4:
                battery_data = struct.unpack_from('<f', sub_payload)[0]
            elif sub_id == SUB_TIMESTAMP and sub_len == 4:
                ts_data = struct.unpack_from('<I', sub_payload)[0]
            else:
                self.get_logger().debug(
                    f'Unknown sub_id=0x{sub_id:02X} len={sub_len}')

        if imu_data     is not None: self._publish_imu(imu_data, now)
        if odom_data    is not None: self._publish_odom(odom_data, now)
        if tof_data     is not None: self._publish_tof(tof_data, now)
        if battery_data is not None: self._publish_battery(battery_data, now)

    # ── Publishers ───────────────────────────────────────────────────────────

    def _publish_imu(self, data, stamp):
        ax, ay, az, gx, gy, gz = data
        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._imu_frame

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x    = gx
        msg.angular_velocity.y    = gy
        msg.angular_velocity.z    = gz

        msg.orientation_covariance[0]          = -1.0
        msg.linear_acceleration_covariance[0]  = 0.01
        msg.linear_acceleration_covariance[4]  = 0.01
        msg.linear_acceleration_covariance[8]  = 0.01
        msg.angular_velocity_covariance[0]     = 0.001
        msg.angular_velocity_covariance[4]     = 0.001
        msg.angular_velocity_covariance[8]     = 0.001

        self._pub_imu.publish(msg)

    def _publish_odom(self, data, stamp):
        x, y, theta, linear, angular, _ = data

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame

        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x  = linear
        msg.twist.twist.angular.z = angular

        msg.pose.covariance[0]   = 0.01
        msg.pose.covariance[7]   = 0.01
        msg.pose.covariance[35]  = 0.05
        msg.twist.covariance[0]  = 0.01
        msg.twist.covariance[35] = 0.05

        self._pub_odom.publish(msg)

        # Broadcast odom→base_link TF
        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id  = self._base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf_broadcaster.sendTransform(tf)

        # ── Integrate wheel positions and publish /joint_states ──
        now_sec = stamp.sec + stamp.nanosec * 1e-9
        if self._last_odom_time is not None:
            dt = now_sec - self._last_odom_time
            if 0 < dt < 0.5:   # guard against stale timestamps
                v_left  = linear - (angular * self.WHEEL_BASE / 2.0)
                v_right = linear + (angular * self.WHEEL_BASE / 2.0)
                self._wheel_left_pos  += (v_left  / self.WHEEL_RADIUS) * dt
                self._wheel_right_pos += (v_right / self.WHEEL_RADIUS) * dt

        self._last_odom_time = now_sec

        js = JointState()
        js.header.stamp = stamp
        js.name         = ['Left_Wheel', 'Right_Wheel']
        js.position     = [self._wheel_left_pos, self._wheel_right_pos]
        js.velocity     = [0.0, 0.0]
        js.effort       = [0.0, 0.0]
        self._pub_joint_states.publish(js)

    def _publish_tof(self, data, stamp):
        left_mm, right_mm, back_mm = data

        def make_range(frame_id, range_mm):
            msg = Range()
            msg.header.stamp    = stamp
            msg.header.frame_id = frame_id
            msg.radiation_type  = Range.INFRARED
            msg.field_of_view   = 0.44
            msg.min_range       = 0.03
            msg.max_range       = 1.20
            msg.range           = float(range_mm) / 1000.0
            return msg

        self._pub_tof_l.publish(make_range('tof_left',  left_mm))
        self._pub_tof_b.publish(make_range('tof_back',  back_mm))
        self._pub_tof_r.publish(make_range('tof_right', right_mm))

    def _publish_battery(self, voltage: float, stamp):
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage      = voltage
        msg.present      = True

        pct = max(0.0, min(1.0, (voltage - 9.0) / (12.6 - 9.0)))
        msg.percentage   = pct

        msg.current             = float('nan')
        msg.charge              = float('nan')
        msg.capacity            = float('nan')
        msg.design_capacity     = float('nan')
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        self._pub_battery.publish(msg)

    # ── cmd_vel → TX ─────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        payload = struct.pack('<3f',
                              msg.linear.x,
                              msg.linear.y,
                              msg.angular.z)
        packet = build_packet(MSG_CMD_VELOCITY, payload)
        try:
            with self._serial_lock:
                self._serial.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Diagnostics ──────────────────────────────────────────────────────────

    def _diag_cb(self):
        p = self._parser
        self.get_logger().info(
            f'[diag] bytes_rx={p.bytes_rx} '
            f'packets_ok={p.packets_ok} '
            f'crc_err={p.crc_errors} '
            f'frame_err={p.frame_errors}'
        )

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._rx_thread.join(timeout=1.0)
        if self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
