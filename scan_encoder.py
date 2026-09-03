import argparse
import serial
import time
from serial.tools import list_ports
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

parser = argparse.ArgumentParser(description="Scan AMT212 encoder IDs.")
parser.add_argument("--port", default=AMT212_PORT, help="Serial port, e.g. /dev/ttyUSB0")
parser.add_argument("--baudrate", type=int, default=AMT212_BAUDRATE)
args = parser.parse_args()

print(f"Scanning all IDs 0-255 on {args.port} at {args.baudrate} baud...")
try:
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.01,
        exclusive=True
    )
    ser.dtr = False
except Exception as e:
    print(f"Port open failed: {e}")
    ports = list(list_ports.comports())
    if ports:
        print("\nAvailable serial ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")
        print(f"\nRetry example: python3 scan_encoder.py --port {ports[0].device}")
    else:
        print("\nNo serial ports found. Check the USB-RS485/USB-serial adapter connection.")
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
