import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ENCODER_ADDR = 0x54

def test_auto_loop():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1, rtscts=False, dsrdtr=False)
    print("Testing with default RTS...")
    for i in range(5):
        ser.reset_input_buffer()
        ser.write(bytes([ENCODER_ADDR]))
        ser.flush()
        buf = ser.read(10)
        print(f"Loop {i}: RX len={len(buf)}, data={buf.hex()}")
        time.sleep(0.05)
    ser.close()

    time.sleep(0.1)

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1, rtscts=False, dsrdtr=False)
    ser.rts = False
    print("Testing with RTS=False...")
    for i in range(5):
        ser.reset_input_buffer()
        ser.write(bytes([ENCODER_ADDR]))
        ser.flush()
        buf = ser.read(10)
        print(f"Loop {i}: RX len={len(buf)}, data={buf.hex()}")
        time.sleep(0.05)
    ser.close()

if __name__ == "__main__":
    test_auto_loop()
