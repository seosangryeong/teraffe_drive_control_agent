import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ENCODER_ADDR = 20

def test_auto_rs485():
    # Test 1: RTS=False static
    print("Testing static RTS=False...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1, rtscts=False, dsrdtr=False)
    ser.rts = False
    ser.dtr = False
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.write(bytes([ENCODER_ADDR]))
    ser.flush()
    buf = ser.read(10)
    print(f"RTS=False: RX len={len(buf)}, data={buf.hex()}")
    ser.close()

    time.sleep(0.1)

    # Test 2: RTS=True static
    print("Testing static RTS=True...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1, rtscts=False, dsrdtr=False)
    ser.rts = True
    ser.dtr = False
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.write(bytes([ENCODER_ADDR]))
    ser.flush()
    buf = ser.read(10)
    print(f"RTS=True: RX len={len(buf)}, data={buf.hex()}")
    ser.close()

if __name__ == "__main__":
    test_auto_rs485()
