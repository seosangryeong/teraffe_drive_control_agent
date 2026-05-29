import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ENCODER_ADDR = 20

def verify_checksum(raw16):
    k1 = (raw16 >> 15) & 1
    k0 = (raw16 >> 14) & 1
    data14 = raw16 & 0x3FFF
    parity_even = 0
    parity_odd = 0
    for i in range(14):
        bit = (data14 >> i) & 1
        if i % 2 == 0:
            parity_even ^= bit
        else:
            parity_odd ^= bit
    return (k1 == (parity_odd ^ 1)) and (k0 == (parity_even ^ 1))

def test_delay(ser, delay):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    ser.rts = False # RTS_TX
    ser.write(bytes([ENCODER_ADDR]))
    ser.flush()
    time.sleep(delay)
    ser.rts = True # RTS_RX
    
    ser.timeout = 0.05
    buf = ser.read(10)
    
    if len(buf) >= 2:
        raw16 = buf[-2] | (buf[-1] << 8)
        if verify_checksum(raw16) and raw16 != 0xFC0A:
            pos = raw16 & 0x3FFF
            deg = pos * 360.0 / 16383.0
            print(f"SUCCESS at delay {delay:.5f}s: raw=0x{raw16:04X}, deg={deg:.2f}, buf={buf.hex()}")
            return True
        elif raw16 != 0xFC0A:
            print(f"Data at {delay:.5f}s: {buf.hex()} (checksum failed)")
    return False

def main():
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=0.1,
        exclusive=True
    )
    ser.rts = True
    ser.dtr = False
    time.sleep(0.5)
    
    for i in range(1, 31):
        delay = i * 0.0001
        print(f"Testing delay: {delay:.5f}s")
        for _ in range(3):
            if test_delay(ser, delay):
                pass
            time.sleep(0.01)

if __name__ == "__main__":
    main()
