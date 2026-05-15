# Agent Notes

이 프로젝트는 고소작업로봇용 ROS2 제어 패키지이다.

## 중요 규칙

- `reference/` 폴더는 참고용이며 수정하지 않는다.
- 새 코드는 `teraffe_drive_control/` 내부에 작성한다.
- 디렉토리 구조는 명령이 있기 전까진 바꾸지않는다.
- 패키지 이름은 `teraffe_drive_control`을 유지한다.
- ROS2 Humble 기준으로 작성한다.
- Python 노드는 `rclpy` 기반으로 작성한다.
- 기존 코드의 동작 방식은 참고하되, 파일 구조와 변수명은 새 프로젝트에 맞게 정리한다.
- 리프트는 ak80-64모터로 동작시키며, 엔코더 값을 통해 모터 리밋을 결정하고, 다이나믹셀로 ak80-64 모터를 보조해준다.


## 참고 방식

- `reference/`의 통신 방식, 클래스 구조, 설정값을 먼저 확인한다.
- 그대로 복사하지 말고 현재 패키지 구조에 맞게 재구성한다.
- 수정 전에는 어떤 파일을 바꿀지 간단히 설명한다.


teraffe_drive_control/
├── config/
│   ├── ak80_64_config.py           # ak80-64 모터 관련 설정           
│   ├── steering_config.py          # 조향모터 관련 설정

│   ├── drive_config.py             # 구동모터 관련 설정
│   ├── dynamixel_config.py         # 다이나믹셀 관련 설정
│   ├── encoder_config.py           # 엔코더 관련 설정
│   
├── hardware/
│   ├── drive_motor.py              # 주행모터 1개 제어
│   ├── steering_motor.py           # 조향모터 1개 제어
│   ├── lift_motor.py               # 리프트모터 1개 제어
│   ├── dynamixel_motor.py          # 보조모터 1개 제어
│   ├── encoder.py                  # 엔코더 1개 제어
│   └── bus.py                      # 통신 관리(485, can)
│
├── modules/
│   ├── drive_module.py              # 조향모터 + 주행모터 
│   └── lift_module.py               # 리프트모터 + 엔코더 + 보조모터 
│
├── controllers/
│   ├── drive_controller.py          # 주행 알고리즘
│   ├── lift_controller.py           # 리프트 상승/하강 알고리즘
│   ├── safety_controller.py         # 안전 조건 감시
│   └── robot_controller.py          # 전체 로봇 동작 통합 제어
│
└── main.py                   # 실행 코드