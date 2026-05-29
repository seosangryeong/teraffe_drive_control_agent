import serial
import time
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.05, exclusive=True)
ser.rts = False # TX
ser.dtr = False
time.sleep(0.1)

print("Testing with delay between write and RTS_RX...")
for delay_us in range(0, 20):
    delay_s = delay_us * 0.0001
    ser.reset_input_buffer()
    ser.rts = False
    ser.write(bytes([20]))
    ser.flush()
    if delay_s > 0:
        time.sleep(delay_s)
    ser.rts = True
    buf = ser.read(10)
    print(f"Delay {delay_s:.4f}s: RX length {len(buf)}, data: {buf.hex()}")

ser.close()
