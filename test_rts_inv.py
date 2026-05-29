import serial
import time
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.02, exclusive=True)

for RTS_TX, RTS_RX in [(False, True), (True, False)]:
    print(f"Testing RTS_TX={RTS_TX}, RTS_RX={RTS_RX}")
    for addr in [20, 0x20, 0x54]:
        for delay_ms in [0.0005, 0.001, 0.002]:
            ser.reset_input_buffer()
            ser.rts = RTS_TX
            ser.write(bytes([addr]))
            ser.flush()
            time.sleep(delay_ms)
            ser.rts = RTS_RX
            
            buf = ser.read(8)
            if len(buf) > 0:
                print(f"  Address {addr} (0x{addr:02X}) delay {delay_ms}s: RX length {len(buf)}, data: {buf.hex()}")

ser.close()
