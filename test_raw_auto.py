import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.1)
ser.reset_input_buffer()
ser.write(bytes([20]))
buf = ser.read(8)
print(f"ID 20 @ 57600: RX length {len(buf)}, data: {buf.hex()}")
ser.close()
