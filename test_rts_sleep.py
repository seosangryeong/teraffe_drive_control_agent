import serial
import time
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.02, exclusive=True)

RTS_TX = False
RTS_RX = True

for addr in [20, 0x20, 0x54]:
    ser.reset_input_buffer()
    ser.rts = RTS_TX
    ser.write(bytes([addr]))
    ser.flush()
    time.sleep(0.001) # 1ms delay for hardware flush!
    ser.rts = RTS_RX
    
    buf = ser.read(8)
    print(f"Address {addr} (0x{addr:02X}): RX length {len(buf)}, data: {buf.hex()}")

ser.close()
