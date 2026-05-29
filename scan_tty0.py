import serial, time
from dynamixel_sdk import PortHandler, PacketHandler
# Check Dynamixel on ttyUSB0
portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = PacketHandler(2.0)
if portHandler.openPort() and portHandler.setBaudRate(57600):
    res = packetHandler.ping(portHandler, 1)
    if res[1] == 0:
        print("Dynamixel is on ttyUSB0")
    portHandler.closePort()

# Check Encoder on ttyUSB0
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    ser.write(bytes([16]))
    ser.flush()
    if len(ser.read(8)) >= 2:
        print("Encoder is on ttyUSB0")
    ser.close()
except:
    pass
