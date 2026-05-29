import time
from dynamixel_sdk import PortHandler, PacketHandler
portHandler = PortHandler('/dev/ttyUSB1')
packetHandler = PacketHandler(2.0)
if portHandler.openPort() and portHandler.setBaudRate(57600):
    dxl_comm_result, dxl_error = packetHandler.ping(portHandler, 1)
    if dxl_comm_result == 0:
        print("Dynamixel found on /dev/ttyUSB1!")
    else:
        print("Ping failed on ttyUSB1")
    portHandler.closePort()
else:
    print("Failed to open ttyUSB1")
