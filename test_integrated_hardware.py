import sys
import time
import tty
import select
import serial
import termios
import subprocess

from dynamixel_sdk import PortHandler, PacketHandler
from teraffe_drive_control_agent.hardware.bus import CanBus
from teraffe_drive_control_agent.hardware.dynamixel_motor import DynamixelMotor
from teraffe_drive_control_agent.hardware.lift_motor import LiftMotor
from teraffe_drive_control_agent.modules.lift_module import LiftModule
from teraffe_drive_control_agent.config.dynamixel_config import PROTOCOL_VERSION
from teraffe_drive_control_agent.config.teraffe_config import DYNAMIXEL_PORT as DEVICENAME, DYNAMIXEL_BAUDRATE as BAUDRATE
from teraffe_drive_control_agent.config.teraffe_config import AMT212_PORT, AMT212_BAUDRATE


def setup_can_interface():
    print("CAN 인터페이스 자동 설정 중...")

    subprocess.run(
        ["sudo", "ip", "link", "set", "can0", "down"],
        stderr=subprocess.DEVNULL,
        check=False,
    )

    subprocess.run(
        ["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"],
        stderr=subprocess.PIPE,
        check=False,
    )

    subprocess.run(
        ["sudo", "ip", "link", "set", "can0", "txqueuelen", "1000"],
        stderr=subprocess.DEVNULL,
        check=False,
    )

    res = subprocess.run(
        ["sudo", "ip", "link", "set", "can0", "up"],
        stderr=subprocess.PIPE,
        check=False,
    )

    if res.returncode != 0:
        print(f"⚠️ CAN 설정 실패: {res.stderr.decode().strip()}")
        print("수동으로 아래 명령을 입력해보세요.")
        print("sudo ip link set can0 down")
        print("sudo ip link set can0 type can bitrate 1000000")
        print("sudo ip link set can0 up")


def main():
    print("--- 통합 하드웨어 단독 테스트 시작 ---")

    setup_can_interface()

    can_bus = CanBus(channel="can0", bitrate=1000000)
    try:
        can_bus.open()
        print("CAN 포트 열기 완료")
    except Exception as e:
        print(f"CAN 포트 열기 실패: {e}")
        return

    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    try:
        if not port_handler.openPort():
            print("Dynamixel 포트 열기 실패")
            can_bus.close()
            return

        if not port_handler.setBaudRate(BAUDRATE):
            print("Dynamixel baudrate 설정 실패")
            port_handler.closePort()
            can_bus.close()
            return

        print("Dynamixel 포트 열기 완료")

    except Exception as e:
        print(f"Dynamixel 포트 초기화 실패: {e}")
        try:
            port_handler.closePort()
        except Exception:
            pass
        can_bus.close()
        return

    try:
        encoder_ser = serial.Serial(AMT212_PORT, AMT212_BAUDRATE, timeout=0.1)
        print("Encoder 포트 열기 완료")
    except Exception as e:
        print(f"Encoder 포트 열기 실패: {e}")
        port_handler.closePort()
        can_bus.close()
        return

    leg_name = "test_leg"

    dxl = DynamixelMotor(
        name=f"{leg_name}_assist",
        dxl_id=3,
        port_handler=port_handler,
        packet_handler=packet_handler,
    )

    ak80 = LiftMotor(
        name=f"{leg_name}_lift",
        motor_id=3,
        can_bus=can_bus,
    )

    module = LiftModule(
        name=f"{leg_name}_lift_module",
        lift_motor=ak80,
        assist_motor=dxl,
        encoder_ser=encoder_ser,
        min_angle_deg=10.0,
        max_angle_deg=350.0,
        encoder_addr=16,
    )

    print("Dynamixel 전류 제어 모드 설정 및 토크 온...")
    dxl.set_current_control_mode()
    dxl.torque_on()

    print("\n--- 통합 구동 테스트 시작 ---")
    print("'a' 키: 전진")
    print("'d' 키: 후진")
    print("'s' 키: 정지")
    print("'q' 키 또는 Ctrl+C: 프로그램 종료\n")

    old_settings = termios.tcgetattr(sys.stdin)

    current_state = None
    action_str = "대기   "

    loop_cnt = 0
    last_dxl_err = 0
    last_dxl_current_A = None

    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)

                if key == "a":
                    current_state = "a"
                    action_str = "전진 중"

                elif key == "d":
                    current_state = "d"
                    action_str = "후진 중"

                elif key == "s":
                    current_state = "s"
                    action_str = "정지   "

                elif key == "q" or key == "\x1b":
                    break

            deg = module.get_position()

            if current_state == "a":
                module.lift_motor.set_erpm(-10000)
                module.assist_motor.assist(-0.02)

            elif current_state == "d":
                module.lift_motor.set_erpm(10000)
                module.assist_motor.assist(0.1)

            elif current_state == "s":
                module.stop()

            loop_cnt += 1

            dxl_current_A = module.assist_motor.read_present_current_A()
            if dxl_current_A is not None:
                last_dxl_current_A = dxl_current_A

            if loop_cnt % 20 == 0:
                err = module.assist_motor.read_hardware_error()
                if err is not None:
                    last_dxl_err = err

            can_errs = module.lift_motor.error_count

            deg_str = f"{deg:6.2f}°" if deg is not None else "None"

            if last_dxl_current_A is not None:
                dxl_current_str = f"{last_dxl_current_A:+.3f} A"
            else:
                dxl_current_str = "None"

            dxl_err_str = f"DXL 에러: {last_dxl_err}" if last_dxl_err > 0 else "DXL 정상"
            can_err_str = f"CAN 드랍: {can_errs}" if can_errs > 0 else "CAN 정상"

            print(
                f"\r각도: {deg_str:>8s} | 상태: {action_str:7s} | "
                f"DXL 전류: {dxl_current_str:>9s} | {can_err_str} | {dxl_err_str}    ",
                end="",
                flush=True,
            )

    except KeyboardInterrupt:
        print("\n\n사용자에 의해 중단되었습니다.")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        print("\n하드웨어를 안전하게 종료합니다...")

        try:
            module.stop()
        except Exception as e:
            print(f"module.stop() 실패: {e}")

        try:
            can_bus.close()
        except Exception as e:
            print(f"CAN 포트 닫기 실패: {e}")

        try:
            port_handler.closePort()
        except Exception as e:
            print(f"Dynamixel 포트 닫기 실패: {e}")

        try:
            encoder_ser.close()
        except Exception as e:
            print(f"Encoder 포트 닫기 실패: {e}")

        print("테스트 종료 및 포트 닫기 완료")


if __name__ == "__main__":
    main()