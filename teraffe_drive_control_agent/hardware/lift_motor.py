from teraffe_drive_control_agent.config.ak80_64_config import *


class LiftMotor:
    def __init__(self, name, motor_id, can_bus):
        self.name = name
        self.id = motor_id
        self.bus = can_bus
        self.error_count = 0  # CAN 전송 에러 카운터 추가

    def make_extended_id(self, packet_id):
        return self.id | (packet_id << 8)

    def int32_to_bytes(self, value):
        value = int(value)

        if value < 0:
            value += 2 ** 32

        return [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]

    def send_packet(self, packet_id, data):
        try:
            self.bus.send_extended(
                arbitration_id=self.make_extended_id(packet_id),
                data=data,
            )
        except Exception as e:
            # CAN 에러 발생 시 카운트 증가
            self.error_count += 1

    def set_erpm(self, erpm):
        if erpm > AK80_64_ERPM_MAX:
            erpm = AK80_64_ERPM_MAX

        if erpm < AK80_64_ERPM_MIN:
            erpm = AK80_64_ERPM_MIN

        self.send_packet(
            CAN_PACKET_SET_RPM,
            self.int32_to_bytes(erpm),
        )

    def set_current_A(self, current_A):
        if current_A > AK80_64_CURRENT_MAX_A:
            current_A = AK80_64_CURRENT_MAX_A

        if current_A < AK80_64_CURRENT_MIN_A:
            current_A = AK80_64_CURRENT_MIN_A

        current_raw = int(current_A * 1000.0)

        self.send_packet(
            CAN_PACKET_SET_CURRENT,
            self.int32_to_bytes(current_raw),
        )

    def stop(self):
        self.set_erpm(0)