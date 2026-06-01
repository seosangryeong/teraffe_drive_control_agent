import time
import serial
from teraffe_drive_control_agent.config.teraffe_config import AMT212_PORT, AMT212_BAUDRATE
from teraffe_drive_control_agent.hardware.encoder import read_position

def main():
    try:
        ser = serial.Serial(
            port=AMT212_PORT,
            baudrate=AMT212_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        )
        ser.rts = True # RTS_RX mode default
        ser.dtr = False
        print("Encoder 포트 열기 완료")
    except Exception as e:
        print(f"Encoder 포트 열기 실패: {e}")
        return

    try:
        print("\n인코더 값을 읽습니다 (종료하려면 Ctrl+C 입력)...")
        while True:
            pos, deg = read_position(ser, encoder_addr=0x10)
            if pos is not None:
                print(f"현재 각도: {deg:7.2f}°  (Raw: {pos})")
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    finally:
        ser.close()
        print("테스트 종료 및 포트 닫기 완료")

if __name__ == "__main__":
    main()
