import serial
import time
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

print(f"Scanning all IDs 0-255 on {AMT212_PORT} at {AMT212_BAUDRATE} baud...")
try:
    ser = serial.Serial(
        port=AMT212_PORT,
        baudrate=AMT212_BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.01,
        exclusive=True
    )
    ser.dtr = False
except Exception as e:
    print(f"Port open failed: {e}")
    exit(1)

found = []
for addr in range(256):
    ser.reset_input_buffer()
    ser.rts = False # TX
    ser.write(bytes([addr]))
    ser.flush()
    ser.rts = True # RX
    
    buf = ser.read(8)
    if len(buf) >= 2:
        found.append((addr, buf))

if found:
    print("\n[SUCCESS] Found devices at following IDs:")
    for addr, buf in found:
        print(f"  ID: {addr} (0x{addr:02X}) - Response len: {len(buf)}, data: {buf.hex()}")
else:
    print("\n[FAIL] No devices responded to any ID from 0 to 255.")

ser.close()
