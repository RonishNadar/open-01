#!/usr/bin/env python3
"""
OPEN-01 OLED Status Display
SH1106 1.3" OLED at I2C address 0x3c
Shows: IP, WiFi signal, ROS2 node status, battery voltage
"""

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import time
import socket
import subprocess
import os
import threading
import types

from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import ImageFont

# ── Config ─────────────────────────────────────────────────────────────────
I2C_PORT    = 1
I2C_ADDRESS = 0x3c
UPDATE_HZ   = 2

ROS_NODES = ['serial_bridge', 'rosbridge_websocket']

# ── Device init ────────────────────────────────────────────────────────────
serial_bus = i2c(port=I2C_PORT, address=I2C_ADDRESS)
device = sh1106(serial_bus, width=128, height=64, rotate=0)

# Fix SH1106 column offset
def _patched_display(self, image):
    assert image.mode == self.mode
    assert image.size == self.size
    image = self.preprocess(image)
    set_page_address = 0xB0
    pixels_per_page = self._w * 8
    buf = bytearray(self._w)
    image_data = list(image.getdata())
    for y in range(0, int(self._pages * pixels_per_page), pixels_per_page):
        self.command(set_page_address, 0x00, 0x10)
        set_page_address += 1
        offsets = [y + self._w * i for i in range(8)]
        for x in range(self._w):
            buf[x] = (
                (image_data[x + offsets[0]] and 0x01) |
                (image_data[x + offsets[1]] and 0x02) |
                (image_data[x + offsets[2]] and 0x04) |
                (image_data[x + offsets[3]] and 0x08) |
                (image_data[x + offsets[4]] and 0x10) |
                (image_data[x + offsets[5]] and 0x20) |
                (image_data[x + offsets[6]] and 0x40) |
                (image_data[x + offsets[7]] and 0x80)
            )
        self.data(list(buf))

device.display = types.MethodType(_patched_display, device)

# ── Fonts ──────────────────────────────────────────────────────────────────
try:
    FONT_SM = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 8)
    FONT_MD = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 10)
    FONT_LG = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf', 12)
except Exception:
    FONT_SM = ImageFont.load_default()
    FONT_MD = ImageFont.load_default()
    FONT_LG = ImageFont.load_default()

# ── Data fetchers ──────────────────────────────────────────────────────────

def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return 'No network'

def get_wifi_signal():
    try:
        out = subprocess.check_output(
            ['iw', 'dev', 'wlan0', 'link'],
            stderr=subprocess.DEVNULL
        ).decode()
        for line in out.split('\n'):
            if 'signal' in line.lower():
                dbm = float(line.strip().split()[1])
                pct = max(0, min(100, int((dbm + 90) / 60 * 100)))
                return f'{pct}%', pct
        return 'N/A', 0
    except Exception:
        return 'N/A', 0

_battery_cache = {'value': 'N/A', 'miss_count': 0}

def get_battery_voltage():
    cache = '/tmp/open01_battery'
    try:
        if os.path.exists(cache):
            age = time.time() - os.path.getmtime(cache)
            if age < 10:
                with open(cache) as f:
                    val = f.read().strip()
                    _battery_cache['value'] = val
                    _battery_cache['miss_count'] = 0
                    return val
        _battery_cache['miss_count'] += 1
        if _battery_cache['miss_count'] > 10:
            _battery_cache['value'] = 'N/A'
        return _battery_cache['value']
    except Exception:
        _battery_cache['miss_count'] += 1
        if _battery_cache['miss_count'] > 10:
            _battery_cache['value'] = 'N/A'
        return _battery_cache['value']

def get_ros_node_status():
    status = {}
    try:
        out = subprocess.check_output(
            ['ros2', 'node', 'list'],
            stderr=subprocess.DEVNULL,
            timeout=2
        ).decode()
        running = out.strip().split('\n')
        for node in ROS_NODES:
            status[node] = any(node in r for r in running)
    except Exception:
        for node in ROS_NODES:
            status[node] = False
    return status

# ── Battery subscriber ─────────────────────────────────────────────────────

def battery_listener():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import BatteryState

        rclpy.init()
        node = Node('oled_battery_listener')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        def cb(msg):
            with open('/tmp/open01_battery', 'w') as f:
                f.write(f'{msg.voltage:.2f}V')

        node.create_subscription(BatteryState, '/battery_state', cb, qos)
        rclpy.spin(node)
    except Exception:
        pass

# ── Startup animation ──────────────────────────────────────────────────────

def startup_animation():
    frames = ['OPEN-01', 'OPEN-01 .', 'OPEN-01 ..', 'OPEN-01 ...']
    for i in range(8):
        with canvas(device) as draw:
            draw.rectangle((0, 0, 127, 63), outline='white', fill='black')
            draw.text((18, 22), frames[i % len(frames)], font=FONT_LG, fill='white')
            draw.text((28, 40), 'BOOTING...', font=FONT_SM, fill='white')
        time.sleep(0.3)

# ── Draw screen ────────────────────────────────────────────────────────────

def draw_screen(ip, wifi_str, wifi_pct, node_status, battery):
    with canvas(device) as draw:

        # Header bar
        draw.rectangle((0, 0, 127, 13), fill='white')
        draw.text((2, 2), 'OPEN-01', font=FONT_SM, fill='black')
        draw.text((55, 2), f'W:{wifi_str}', font=FONT_SM, fill='black')

        # Battery bar (top right, 3S Li-Ion: 9V=0%, 12.6V=100%)
        bx = 100
        draw.rectangle((bx, 2, bx + 22, 10), outline='black')
        draw.rectangle((bx + 22, 4, bx + 24, 8), fill='black')
        bat_fill = 0
        if battery != 'N/A':
            try:
                v = float(battery.replace('V', ''))
                bat_fill = max(0, min(20, int((v - 9.0) / (12.6 - 9.0) * 20)))
            except Exception:
                bat_fill = 0
        if bat_fill > 0:
            draw.rectangle((bx + 1, 3, bx + 1 + bat_fill, 9), fill='black')

        # IP
        draw.text((0, 16), 'IP:', font=FONT_SM, fill='white')
        draw.text((18, 15), ip, font=FONT_MD, fill='white')

        # Battery voltage
        draw.text((0, 29), f'BAT: {battery}', font=FONT_SM, fill='white')

        # Divider
        draw.line((0, 40, 127, 40), fill='white')

        # Node status
        draw.text((0, 43), 'NODES:', font=FONT_SM, fill='white')
        col, row = 0, 53
        for name, alive in node_status.items():
            marker = '+' if alive else '!'
            label = f'[{marker}]{name[:8]}'
            draw.text((col, row), label, font=FONT_SM, fill='white')
            col += len(label) * 6 + 2
            if col > 90:
                col = 0
                row += 11

# ── Main ───────────────────────────────────────────────────────────────────

def main():
    threading.Thread(target=battery_listener, daemon=True).start()
    startup_animation()

    ip = get_ip()
    last_ip_check = time.time()

    while True:
        try:
            now = time.time()
            if now - last_ip_check > 30:
                ip = get_ip()
                last_ip_check = now

            wifi_str, wifi_pct = get_wifi_signal()
            node_status = get_ros_node_status()
            battery = get_battery_voltage()
            draw_screen(ip, wifi_str, wifi_pct, node_status, battery)

        except Exception as e:
            try:
                with canvas(device) as draw:
                    draw.text((0, 0), 'ERR:', font=FONT_SM, fill='white')
                    draw.text((0, 12), str(e)[:20], font=FONT_SM, fill='white')
            except Exception:
                pass

        time.sleep(1.0 / UPDATE_HZ)

if __name__ == '__main__':
    main()
