import time
from dynamixel_sdk import PortHandler, PacketHandler
from teraffe_drive_control_agent.hardware.dynamixel_motor import DynamixelMotor
from teraffe_drive_control_agent.config.dynamixel_config import PROTOCOL_VERSION
from teraffe_drive_control_agent.config.teraffe_config import DYNAMIXEL_PORT as DEVICENAME, DYNAMIXEL_BAUDRATE as BAUDRATE

def main():
    print("--- 다이나믹셀 연속 왕복 테스트 (ID: 102) ---")
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    if port_handler.openPort() and port_handler.setBaudRate(BAUDRATE):
        print("Dynamixel 포트 열기 완료")
    else:
        print("Dynamixel 포트 열기 실패")
        return

    dxl = DynamixelMotor(
        name="test_assist", 
        dxl_id=102, 
        port_handler=port_handler, 
        packet_handler=packet_handler
    )
    
    try:
        # 안전하게 동작 모드 설정 및 토크 온
        dxl.set_current_control_mode()
        dxl.torque_on()
        
        print("\n전류 +0.05A / -0.05A를 3초 간격으로 반복합니다.")
        print("멈추려면 Ctrl+C를 누르세요.")

        while True:
            print("전류 +0.05A(extend 방향)")
            dxl.assist(0.05)
            time.sleep(3)

            print("전류 -0.05A(retract 방향)")
            dxl.assist(-0.05)
            time.sleep(3)
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    finally:
        dxl.stop_assist()
        dxl.torque_off()
        port_handler.closePort()
        print("테스트 종료 및 포트 닫기 완료")

if __name__ == "__main__":
    main()
