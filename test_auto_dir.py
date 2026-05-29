import serial
import time
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.1, rtscts=False, dsrdtr=False, exclusive=True)
ser.rts = False
ser.dtr = False
time.sleep(0.1)

for addr in [20, 0x14, 0x54]:
    ser.reset_input_buffer()
    ser.write(bytes([addr]))
    buf = ser.read(8)
    print(f"ADDR {addr}: RX len {len(buf)}, data: {buf.hex()}")
    
ser.close()
