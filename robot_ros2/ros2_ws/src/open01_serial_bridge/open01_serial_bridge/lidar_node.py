#!/usr/bin/env python3
"""
OPEN-01 LDS-02 Lidar Node
Reads LDS-02 (LD08) serial protocol and publishes sensor_msgs/LaserScan.

LDS-02 packet format (42 bytes):
  [0xFA][INDEX][SPEED_L][SPEED_H][DATA x6][CHECKSUM_L][CHECKSUM_H]
  Each DATA block (6 bytes): [DIST_L][DIST_H][INTENSITY_L][INTENSITY_H][reserved x2]
  INDEX: 0xA0..0xF9 (90 packets per revolution, each covers 4 degrees)
  SPEED: RPM as uint16 little-endian, in units of 0.01 RPM
  DIST:  uint16 little-endian, in mm
"""

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial
import struct

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

# ── Protocol constants ────────────────────────────────────────────────────────
PACKET_START    = 0xFA
PACKET_LEN      = 42        # bytes per packet
INDEX_MIN       = 0xA0      # first packet index
INDEX_MAX       = 0xDB      # last packet index (90 packets total)
PACKETS_PER_REV = 90
ANGLES_PER_PKT  = 4         # each packet covers 4 degrees
TOTAL_ANGLES    = 360       # full scan
BAUD_RATE       = 230400

# ── Checksum ─────────────────────────────────────────────────────────────────

def compute_checksum(data: bytes) -> int:
    """LDS-02 checksum: sum of bytes 0..39, take lower 16 bits."""
    s = sum(data[:40])
    return ((~s & 0xFF) | ((~s & 0xFF) << 8))


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar')

        # ── Parameters ──
        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('frame_id',     'laser')
        self.declare_parameter('min_range',    0.12)    # m
        self.declare_parameter('max_range',    3.5)     # m
        self.declare_parameter('publish_rpm',  False)

        port           = self.get_parameter('port').value
        self._frame_id = self.get_parameter('frame_id').value
        self._min_range = self.get_parameter('min_range').value
        self._max_range = self.get_parameter('max_range').value
        publish_rpm    = self.get_parameter('publish_rpm').value

        # ── Serial ──
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                timeout=0.02,
            )
            self.get_logger().info(f'Opened lidar port {port} @ {BAUD_RATE} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open lidar port {port}: {e}')
            raise

        # ── Publishers ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self._pub_scan = self.create_publisher(LaserScan, '/scan', sensor_qos)
        if publish_rpm:
            self._pub_rpm = self.create_publisher(Float32, '/lidar/rpm', sensor_qos)
        else:
            self._pub_rpm = None

        # ── Scan buffer: 360 slots (one per degree) ──
        self._ranges      = [float('inf')] * TOTAL_ANGLES
        self._intensities = [0.0] * TOTAL_ANGLES
        self._last_rpm    = 0.0

        # ── RX buffer ──
        self._rx_buf = deque()
        self._rx_buf_lock = threading.Lock()

        # ── Stats ──
        self._packets_ok  = 0
        self._packets_err = 0
        self._scans_pub   = 0

        # ── RX thread ──
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_thread_fn, daemon=True)
        self._rx_thread.start()

        # ── Parse timer (100Hz) ──
        self._parse_timer = self.create_timer(0.01, self._parse_timer_cb)

        # ── Diagnostics ──
        self.create_timer(5.0, self._diag_cb)

        self.get_logger().info('lidar node started')

    # ── RX thread ────────────────────────────────────────────────────────────

    def _rx_thread_fn(self):
        buf = bytearray()
        while self._running:
            try:
                data = self._serial.read(256)
                if data:
                    with self._rx_buf_lock:
                        self._rx_buf.append(bytes(data))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    # ── Parse timer ──────────────────────────────────────────────────────────

    def _parse_timer_cb(self):
        chunks = []
        with self._rx_buf_lock:
            while self._rx_buf:
                chunks.append(self._rx_buf.popleft())

        if not chunks:
            return

        # Accumulate into local buffer
        if not hasattr(self, '_parse_buf'):
            self._parse_buf = bytearray()
        for chunk in chunks:
            self._parse_buf.extend(chunk)

        # Extract complete packets
        buf = self._parse_buf
        i = 0
        while i < len(buf):
            # Find next start byte
            if buf[i] != PACKET_START:
                i += 1
                continue

            # Need full packet
            if i + PACKET_LEN > len(buf):
                break

            packet = buf[i:i + PACKET_LEN]

            # Validate second byte is valid index
            index = packet[1]
            if index < INDEX_MIN or index > INDEX_MAX:
                i += 1
                continue

            # Validate checksum
            expected_crc = compute_checksum(packet)
            actual_crc   = struct.unpack_from('<H', packet, 40)[0]
            if expected_crc != actual_crc:
                self._packets_err += 1
                i += 1
                continue

            # Valid packet — parse it
            self._parse_packet(packet)
            self._packets_ok += 1
            i += PACKET_LEN

        self._parse_buf = buf[i:]

    def _parse_packet(self, packet: bytes):
        index = packet[1]
        speed_rpm = struct.unpack_from('<H', packet, 2)[0] / 100.0
        self._last_rpm = speed_rpm

        # Base angle for this packet (degrees)
        base_angle = (index - INDEX_MIN) * ANGLES_PER_PKT

        # Parse 6 measurements
        for j in range(6):
            offset = 4 + j * 6
            dist_mm   = struct.unpack_from('<H', packet, offset)[0]
            intensity = struct.unpack_from('<H', packet, offset + 2)[0]

            angle_deg = (base_angle + j) % TOTAL_ANGLES
            dist_m    = dist_mm / 1000.0

            # Filter invalid readings
            if dist_mm == 0 or dist_m < self._min_range or dist_m > self._max_range:
                self._ranges[angle_deg]      = float('inf')
                self._intensities[angle_deg] = 0.0
            else:
                self._ranges[angle_deg]      = dist_m
                self._intensities[angle_deg] = float(intensity)

        # Publish scan when we receive the last packet of a revolution
        if index == INDEX_MAX:
            self._publish_scan()

    def _publish_scan(self):
        now = self.get_clock().now().to_msg()

        msg = LaserScan()
        msg.header.stamp    = now
        msg.header.frame_id = self._frame_id

        msg.angle_min       = 0.0
        msg.angle_max       = 2.0 * math.pi
        msg.angle_increment = math.radians(1.0)   # 1 degree per sample
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / 5.0           # ~5Hz full scans
        msg.range_min       = self._min_range
        msg.range_max       = self._max_range

        msg.ranges      = list(self._ranges)
        msg.intensities = list(self._intensities)

        self._pub_scan.publish(msg)
        self._scans_pub += 1

        if self._pub_rpm:
            rpm_msg = Float32()
            rpm_msg.data = self._last_rpm
            self._pub_rpm.publish(rpm_msg)

    # ── Diagnostics ──────────────────────────────────────────────────────────

    def _diag_cb(self):
        self.get_logger().info(
            f'[lidar diag] packets_ok={self._packets_ok} '
            f'packets_err={self._packets_err} '
            f'scans_pub={self._scans_pub} '
            f'rpm={self._last_rpm:.1f}'
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
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
