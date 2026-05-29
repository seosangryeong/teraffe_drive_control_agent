import serial, time
from dynamixel_sdk import PortHandler, PacketHandler

def check_dynamixel(port):
    portHandler = PortHandler(port)
    packetHandler = PacketHandler(2.0)
    if portHandler.openPort() and portHandler.setBaudRate(57600):
        res = packetHandler.ping(portHandler, 1)
        portHandler.closePort()
        if res[1] == 0:
            return True
    return False

def check_encoder(port):
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        ser.write(bytes([16]))
        ser.flush()
        data = ser.read(8)
        ser.close()
        if len(data) >= 2:
            return True
    except:
        pass
    return False

print(f"ttyUSB0 - Dynamixel: {check_dynamixel('/dev/ttyUSB0')}, Encoder: {check_encoder('/dev/ttyUSB0')}")
print(f"ttyUSB1 - Dynamixel: {check_dynamixel('/dev/ttyUSB1')}, Encoder: {check_encoder('/dev/ttyUSB1')}")
