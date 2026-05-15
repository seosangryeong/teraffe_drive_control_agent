import rclpy
from rclpy.node import Node
import serial
import time

from dynamixel_sdk import PortHandler, PacketHandler

from teraffe_drive_control_agent.hardware.bus import CanBus
from teraffe_drive_control_agent.hardware.dynamixel_motor import DynamixelMotor
from teraffe_drive_control_agent.hardware.lift_motor import LiftMotor
from teraffe_drive_control_agent.modules.lift_module import LiftModule

from teraffe_drive_control_agent.config.dynamixel_config import DEVICENAME, BAUDRATE, PROTOCOL_VERSION, DYNAMIXEL_IDS
from teraffe_drive_control_agent.config.ak80_64_config import AK80_64_IDS
from teraffe_drive_control_agent.config.encoder_config import AMT212_PORT, AMT212_BAUDRATE

class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')
        
        self.get_logger().info("Lift Controller 하드웨어 셋업 시작...")

        self.is_can_connected = False
        self.is_dxl_connected = False
        self.is_enc_connected = False

        # ==========================================
        # 1. 통신 버스 및 포트 초기화
        # ==========================================
        self.can_bus = CanBus(channel="can0", bitrate=1000000)
        try:
            self.can_bus.open()
            self.is_can_connected = True
            self.get_logger().info("CAN 포트(can0) 열기 성공")
        except Exception as e:
            self.get_logger().error(f"CAN 포트 열기 실패 (하드웨어 연결 확인 요망): {e}")

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        try:
            if self.port_handler.openPort():
                self.get_logger().info(f"Dynamixel 포트({DEVICENAME}) 열기 성공")
                if self.port_handler.setBaudRate(BAUDRATE):
                    self.get_logger().info(f"Dynamixel 보드레이트({BAUDRATE}) 설정 성공")
                    self.is_dxl_connected = True
            else:
                self.get_logger().error("Dynamixel 포트 열기 실패")
        except Exception as e:
            self.get_logger().error(f"Dynamixel 포트 예외 발생 (하드웨어 연결 확인 요망): {e}")

        try:
            self.encoder_ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.1)
            self.get_logger().info(f"Encoder 포트({AMT212_PORT}) 열기 성공")
            self.is_enc_connected = True
        except Exception as e:
            self.get_logger().error(f"Encoder 포트 열기 실패: {e}")
            self.encoder_ser = None


        # ==========================================
        # 2. LiftModule 조립 (4개의 다리 모두 셋업)
        # ==========================================
        self.lift_modules = {}
        
        leg_names = [
            "front_left",
            "front_right",
            "rear_left",
            "rear_right"
        ]
        
        for name in leg_names:
            # 다이나믹셀 객체 생성
            dxl = DynamixelMotor(
                name=f"{name}_assist",
                dxl_id=DYNAMIXEL_IDS.get(name, 1),
                port_handler=self.port_handler,
                packet_handler=self.packet_handler
            )
            
            # LiftMotor 객체 생성
            ak80 = LiftMotor(
                name=f"{name}_lift",
                motor_id=AK80_64_IDS.get(f"{name}_lift", 100),
                can_bus=self.can_bus
            )
            
            # LiftModule 조립 및 저장
            module = LiftModule(
                name=f"{name}_lift_module",
                lift_motor=ak80,
                assist_motor=dxl,
                encoder_ser=self.encoder_ser,
                min_angle_deg=10.0,  # 테스트용 최소 리밋 각도
                max_angle_deg=150.0  # 테스트용 최대 리밋 각도
            )
            
            self.lift_modules[name] = module
            
            # 외부 컨트롤러(ex: robot_controller.py)에서 직관적으로 접근할 수 있도록 속성 정의
            if name == "front_left":
                self.lift_fl = module
            elif name == "front_right":
                self.lift_fr = module
            elif name == "rear_left":
                self.lift_rl = module
            elif name == "rear_right":
                self.lift_rr = module
                
            self.get_logger().info(f"{name} LiftModule 조립 완료!")

        # ==========================================
        # 3. 제어 및 감시 타이머
        # ==========================================
        # 0.1초마다 리밋을 검사하는 안전 감시 루프
        self.timer = self.create_timer(0.1, self.timer_callback)


    # ==========================================
    # 외부 알고리즘 연동용 Helper 함수들
    # ==========================================
    def get_connection_status(self):
        """하드웨어 연결 상태를 반환합니다."""
        return {
            "CAN": self.is_can_connected,
            "DXL": self.is_dxl_connected,
            "ENC": self.is_enc_connected
        }

    def get_lift(self, leg_name):
        """특정 다리의 LiftModule 객체를 반환합니다. (예: 'front_left')"""
        return self.lift_modules.get(leg_name)

    def get_leg_position(self, leg_name):
        """특정 다리의 현재 엔코더 각도를 반환합니다."""
        module = self.get_lift(leg_name)
        if module:
            return module.get_position()
        return None

    def extend_leg(self, leg_name, erpm):
        """특정 다리의 길이를 늘립니다."""
        module = self.get_lift(leg_name)
        if module:
            return module.extend(erpm)
        return False

    def retract_leg(self, leg_name, erpm):
        """특정 다리의 길이를 줄립니다."""
        module = self.get_lift(leg_name)
        if module:
            return module.retract(erpm)
        return False

    def stop_all(self):
        """모든 다리의 동작을 정지합니다."""
        for module in self.lift_modules.values():
            module.stop()

    def timer_callback(self):
        """주기적으로 모든 LiftModule의 리밋 상태를 체크합니다."""
        for name, module in self.lift_modules.items():
            limit_hit = module.check_and_stop_if_limit()
            if limit_hit:
                self.get_logger().warn(f"[{name}] 하드웨어 리밋 도달! 동작 정지됨.")

    def destroy_node(self):
        """노드 종료 시 모터 정지 및 포트 닫기"""
        self.get_logger().info("Lift Controller 종료 중...")
        self.stop_all()
            
        self.can_bus.close()
        self.port_handler.closePort()
        
        if self.encoder_ser:
            self.encoder_ser.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
