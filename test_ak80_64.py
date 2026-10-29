import time
import subprocess
from teraffe_drive_control_agent.hardware.bus import CanBus
from teraffe_drive_control_agent.hardware.lift_motor import LiftMotor


def setup_can_interface(channel="can0", bitrate=1000000):
    subprocess.run(["sudo", "ip", "link", "set", channel, "down"], check=False)
    subprocess.run(
        ["sudo", "ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)],
        check=True
    )
    subprocess.run(["sudo", "ip", "link", "set", channel, "up"], check=True)


def main():
    print("--- AK80-64 모터 연속 회전 테스트 (ID: 1) ---")

    try:
        setup_can_interface(channel="can0", bitrate=1000000)
        print("CAN 인터페이스 설정 완료")
    except Exception as e:
        print(f"CAN 인터페이스 설정 실패: {e}")
        return

    can_bus = CanBus(channel="can0", bitrate=1000000)

    try:
        can_bus.open()
        print("CAN 포트 열기 완료")
    except Exception as e:
        print(f"CAN 포트 열기 실패: {e}")
        return

    ak80 = LiftMotor(name="test_lift", motor_id=104, can_bus=can_bus)

    try:
        print("\nERPM 5000으로 계속 회전합니다.")
        print("멈추려면 Ctrl+C를 누르세요.")

        while True:
            ak80.set_erpm(5000)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다. 정지합니다.")

    finally:
        try:
            ak80.stop()
            time.sleep(0.2)
        except Exception as e:
            print(f"정지 명령 실패: {e}")

        try:
            can_bus.close()
        except Exception as e:
            print(f"CAN 포트 닫기 실패: {e}")

        print("테스트 종료 및 포트 닫기 완료")


if __name__ == "__main__":
    main()