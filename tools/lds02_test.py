import serial, time

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.reset_input_buffer()

buf = bytearray()
t0 = time.time()
while time.time() - t0 < 5:
    data = ser.read(ser.in_waiting or 256)
    if data:
        buf.extend(data)

ser.close()

print("bytes:", len(buf))
print("first 128:", " ".join(f"{b:02X}" for b in buf[:128]))
print("54 2C headers:", buf.count(bytes([0x54, 0x2C])))