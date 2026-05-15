import time
from teraffe_drive_control_agent.hardware.encoder import read_position

class LiftModule:
    """
    리프트 제어 모듈
    - AK80-64 모터(주 동력)와 다이나믹셀(보조 동력)을 함께 사용하여 길이를 조절합니다.
    - 하드웨어 엔코더 값을 읽어 지정된 길이(각도) 범위를 벗어나지 않도록 제한(Limit)합니다.
    """
    def __init__(
        self,
        name,
        lift_motor,
        assist_motor,
        encoder_ser,
        min_angle_deg,
        max_angle_deg,
    ):
        self.name = name
        
        # 하드웨어 객체
        self.lift_motor = lift_motor          # AK8064Motor 인스턴스
        self.assist_motor = assist_motor      # DynamixelMotor 인스턴스
        self.encoder_ser = encoder_ser        # serial.Serial 인스턴스 (인코더 통신용)
        
        # 동작 한계 (소프트웨어 리밋)
        self.min_angle_deg = min_angle_deg
        self.max_angle_deg = max_angle_deg

    def get_position(self):
        """
        엔코더를 통해 현재 각도를 읽어옵니다.
        읽기 실패 시 None을 반환합니다.
        """
        pos, deg = read_position(self.encoder_ser)
        return deg

    def extend(self, erpm):
        """
        리프트 길이를 늘립니다. (상승)
        - 다이나믹셀(보조모터): -0.05A로 회전
        - AK80-64(리프트모터): 지정된 ERPM으로 회전
        """
        deg = self.get_position()
        
        # 1. 엔코더 한계값 체크 (최대 길이 도달 여부)
        if deg is not None and deg >= self.max_angle_deg:
            print(f"[{self.name}] ⚠️ 최대 길이(Limit) 도달 (현재: {deg:.2f}° / 최대: {self.max_angle_deg}°). 동작을 중지합니다.")
            self.stop()
            return False

        # 2. 보조 모터(다이나믹셀) 제어 (0.05A - AK80-64와 부호 일치)
        self.assist_motor.assist(0.05)
        
        # 3. 주 구동 모터(AK80-64) 제어 (양수 방향)
        self.lift_motor.set_erpm(abs(erpm))
        
        return True

    def retract(self, erpm):
        """
        리프트 길이를 줄입니다. (하강)
        - 다이나믹셀(보조모터): 0.1A로 회전
        - AK80-64(리프트모터): 역방향 ERPM으로 회전
        """
        deg = self.get_position()

        # 1. 엔코더 한계값 체크 (최소 길이 도달 여부)
        if deg is not None and deg <= self.min_angle_deg:
            print(f"[{self.name}] ⚠️ 최소 길이(Limit) 도달 (현재: {deg:.2f}° / 최소: {self.min_angle_deg}°). 동작을 중지합니다.")
            self.stop()
            return False

        # 2. 보조 모터(다이나믹셀) 제어 (-0.1A - AK80-64와 부호 일치)
        self.assist_motor.assist(-0.1)

        # 3. 주 구동 모터(AK80-64) 제어 (음수 방향)
        self.lift_motor.set_erpm(-abs(erpm))
        
        return True

    def stop(self):
        """
        모든 모터의 동작을 중지합니다.
        """
        self.lift_motor.stop()
        self.assist_motor.stop_assist()
        
    def check_and_stop_if_limit(self):
        """
        주기적인 루프에서 엔코더 리밋을 감시할 때 사용할 수 있는 헬퍼 함수
        """
        deg = self.get_position()
        if deg is not None:
            if deg <= self.min_angle_deg or deg >= self.max_angle_deg:
                self.stop()
                return True # 리밋에 걸려 정지됨
        return False
