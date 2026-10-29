import serial
import time

ENCODER_ADDR = 0x54
RETRY_CNT = 5

RTS_TX = False
RTS_RX = True


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


def read_position(ser, encoder_addr=ENCODER_ADDR):
    if ser is None or not ser.is_open:
        return None, None

    previous_timeout = ser.timeout
    try:
        try:
            for attempt in range(RETRY_CNT):
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.rts = RTS_TX
                ser.write(bytes([encoder_addr]))
                ser.flush()
                ser.rts = RTS_RX

                # Read up to 8 bytes to absorb RTS toggle glitches.
                ser.timeout = 0.01
                buf = ser.read(8)
                if len(buf) >= 2:
                    raw16 = buf[-2] | (buf[-1] << 8)
                    if verify_checksum(raw16):
                        pos = raw16 & 0x3FFF
                        return pos, pos * 360.0 / 16383.0
                time.sleep(0.01)
            return None, None
        finally:
            ser.timeout = previous_timeout
            if ser.is_open:
                ser.rts = RTS_RX
    except (serial.SerialException, OSError):
        return None, None
