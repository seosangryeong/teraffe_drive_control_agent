import serial
import serial.rs485
import time
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.05, exclusive=True)
ser.rs485_mode = serial.rs485.RS485Settings(rts_level_for_tx=False, rts_level_for_rx=True, delay_before_tx=None, delay_before_rx=None)

for addr in [20, 0x20, 0x54]:
    ser.reset_input_buffer()
    ser.write(bytes([addr]))
    buf = ser.read(8)
    print(f"HW RS485 mode (TX=False) - Address {addr} (0x{addr:02X}): RX length {len(buf)}, data: {buf.hex()}")

ser.rs485_mode = serial.rs485.RS485Settings(rts_level_for_tx=True, rts_level_for_rx=False, delay_before_tx=None, delay_before_rx=None)
for addr in [20, 0x20, 0x54]:
    ser.reset_input_buffer()
    ser.write(bytes([addr]))
    buf = ser.read(8)
    print(f"HW RS485 mode (TX=True) - Address {addr} (0x{addr:02X}): RX length {len(buf)}, data: {buf.hex()}")

ser.close()
