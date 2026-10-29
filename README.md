# Teraffe Drive Control Agent

**Teraffe 로봇의 모터·엔코더 상태 확인과 리프트 제어를 위한 ROS 2 패키지다.**

AK80-64, Dynamixel XH430, AMT212를 연결해 브라우저 또는 데스크톱 UI에서 모니터링하고 제어한다. ROS 토픽 기반 원격 리프트 제어와 외부 주행·조향 상태 모니터도 제공한다.

| 하드웨어 | 통신 | 주요 기능 |
| --- | --- | --- |
| AK80-64 | SocketCAN | ERPM 제어, 위치·전류·온도·오류 모니터링 |
| Dynamixel XH430 | RS-485 / Protocol 2.0 | 전류 제어, 토크 설정, 상태 모니터링 |
| AMT212 | RS-485 | 절대 각도 읽기, 체크섬 검증 |

## 목차

1. [설치](#1-설치)
2. [하드웨어 연결과 설정](#2-하드웨어-연결과-설정)
3. [웹 UI 실행](#3-웹-ui-실행)
4. [데스크톱 UI 실행](#4-데스크톱-ui-실행)
5. [ROS 리프트 제어](#5-ros-리프트-제어)
6. [주행·조향 상태 모니터](#6-주행조향-상태-모니터)
7. [장치 검색과 진단](#7-장치-검색과-진단)
8. [개발과 검증](#8-개발과-검증)
9. [문제 해결](#9-문제-해결)

## 1. 설치

### 기준 환경

| 항목 | 환경 |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| Python | 3.10 |
| 빌드 | `ament_python`, `colcon` |
| CAN | Linux SocketCAN, `1 Mbps` |
| 시리얼 | 기본 `115200 baud` |
| UI | 웹 브라우저, PyQt5 |

아래 명령은 Bash와 `~/ros2_ws`를 기준으로 한다. 설치 순서는 **ROS 설치 → 저장소 복제 → 의존성 설치 → 빌드**다.

### 1-1. ROS 2 Humble 준비

ROS가 설치되어 있다면 환경만 적용하고 다음 단계로 이동한다.

```bash
source /opt/ros/humble/setup.bash
```

<details>
<summary><strong>ROS가 없는 Ubuntu 22.04에서 처음 설치하는 경우</strong></summary>

UTF-8 로케일과 Ubuntu Universe 저장소를 준비한다.

```bash
sudo apt update
sudo apt install -y locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```

ROS 공식 apt 저장소 설정 패키지를 설치한다. 아래의 `jammy`는 Ubuntu 22.04 기준이다.

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base
source /opt/ros/humble/setup.bash
```

RViz 등 ROS 데스크톱 도구도 필요하면 `ros-humble-desktop`을 설치한다. 자세한 내용은 [ROS 2 Humble 공식 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)를 참고한다.

</details>

### 1-2. 빌드 도구 설치 및 저장소 복제

```bash
sudo apt install -y \
  git python3-setuptools python3-colcon-common-extensions \
  python3-rosdep can-utils iproute2

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/seosangryeong/teraffe_drive_control_agent.git
```

이미 복제한 저장소가 있다면 해당 경로를 사용한다.

### 1-3. 의존성 설치

`rosdep`을 처음 사용하는 시스템에서만 초기화한다.

```bash
sudo rosdep init
```

초기화가 완료된 시스템에서는 아래부터 실행한다.

```bash
source /opt/ros/humble/setup.bash
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src/teraffe_drive_control_agent \
  --ignore-src -r -y --rosdistro humble
```

`package.xml`에 선언된 ROS 메시지, Python CAN·시리얼·Qt 라이브러리, Dynamixel SDK를 설치한다. 시스템 Python을 사용하며 데스크톱 UI는 GUI 세션에서 실행한다. 웹 UI도 공통 상태 모듈을 사용하므로 PyQt5 의존성을 함께 설치한다.

### 1-4. 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select teraffe_drive_control_agent --symlink-install
source install/setup.bash
ros2 pkg executables teraffe_drive_control_agent
```

실행 파일 목록에 `robot_hardware_web_ui`, `robot_tension_web_ui`, `robot_controller` 등이 나오면 패키지 등록이 완료된 상태다.

> **새 터미널에서는 아래 두 줄을 먼저 실행한다.**
>
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/ros2_ws/install/setup.bash
> ```

## 2. 하드웨어 연결과 설정

### 연결 방식 선택

| 실행 경로 | 시리얼 구성 | 설정 위치 |
| --- | --- | --- |
| 통합 웹 UI / 공용 포트 데스크톱 UI | DXL + 엔코더가 시리얼 포트 하나를 공유 | CLI 인자, 웹 UI의 `MODULES` |
| ROS 리프트 제어 | DXL과 엔코더를 서로 다른 시리얼 포트에 연결 | `config/teraffe_config.py` |
| 장치별 모니터 | 확인할 장치의 포트에 연결 | CLI 인자 |

웹 UI의 기본 연결은 다음과 같다.

```text
Ubuntu PC
├── SocketCAN 어댑터 ── can0 ── AK80-64
└── RS-485 어댑터 ── /dev/ttyUSB0 ── Dynamixel + AMT212
```

### 2-1. 시리얼 포트와 권한

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$USER"
```

그룹 추가 후에는 **로그아웃하고 다시 로그인**한다. USB 번호가 바뀌는 환경에서는 `/dev/serial/by-id/`의 고정 경로를 실행 인자로 사용할 수 있다.

같은 시리얼 포트를 사용하는 UI와 진단 스크립트는 하나씩 실행한다.

### 2-2. CAN 설정

SocketCAN 어댑터를 연결한 뒤 인터페이스 이름과 bitrate를 설정한다.

```bash
ip -brief link
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

`can0`가 존재해야 하며 모터의 bitrate도 `1 Mbps`여야 한다. 인터페이스 이름이 다르면 아래 실행 명령의 CAN 채널도 변경한다.

```bash
# 상태 프레임 수신 확인. Ctrl+C로 종료한다.
candump can0
```

웹 UI는 시작과 연결 재설정 시 CAN 인터페이스를 다시 설정한다. 웹 UI를 실행할 터미널에서 `sudo -v`로 인증을 준비한다. 다른 프로그램이 같은 CAN을 사용 중이면 재설정 동안 통신이 잠시 끊길 수 있다.

### 2-3. 장치 ID

**웹 UI의 기본 매핑**은 [robot_hardware_web_ui.py](teraffe_drive_control_agent/robot_hardware_web_ui.py)의 `MODULES`에서 관리한다. Tension UI도 같은 매핑을 사용한다.

| 모듈 | 위치 | AK CAN ID | DXL ID | 엔코더 요청 바이트 |
| --- | --- | --- | --- | --- |
| `FL` | 전방 좌측 | `101` | `111` | `0x1C` |
| `FR` | 전방 우측 | `102` | `112` | `0x2C` |
| `RL` | 후방 좌측 | `103` | `113` | `0x3C` |
| `RR` | 후방 우측 | `104` | `114` | `0x4C` |

**ROS 리프트 제어의 기본 매핑**은 [teraffe_config.py](teraffe_drive_control_agent/config/teraffe_config.py)에서 관리한다.

| 항목 | 기본 설정 |
| --- | --- |
| DXL | `/dev/ttyUSB0`, `115200 baud`, 좌전·우전·좌후·우후 ID `1 / 2 / 3 / 4` |
| 엔코더 | `/dev/ttyUSB1`, `115200 baud`, 요청 바이트 `0x1C / 0x2C / 0x3C / 0x4C` |
| AK 리프트 | `can0`, `1 Mbps`, 좌전·우전·좌후·우후 ID `1 / 2 / 3 / 4` |

선택한 실행 경로의 설정을 실제 장치에 맞춘다. 코드의 ID 설정은 장치에 자동으로 기록되지 않는다. 엔코더 요청 바이트는 코드가 그대로 전송하는 값이다.

## 3. 웹 UI 실행

### 3-1. 통합 하드웨어 UI

**모듈별 상태를 보고 AK 속도와 DXL 전류를 직접 조절할 때 사용한다.** 별도 ROS 제어 노드 없이 실행한다.

```bash
sudo -v
ros2 run teraffe_drive_control_agent robot_hardware_web_ui \
  --serial-port /dev/ttyUSB0 \
  --dxl-baudrate 115200 --encoder-baudrate 115200 \
  --can-channel can0 --can-bitrate 1000000 \
  --host 127.0.0.1 --port 8080
```

**접속 주소: <http://127.0.0.1:8080>**

1. 연결 설정에서 포트·baudrate·CAN 채널을 확인한다. 변경한 경우 `Apply Connections`를 누른다.
2. FL / FR / RL / RR의 AK·DXL·엔코더 응답을 확인한다.
3. 제어할 모듈을 선택하고 아래 버튼으로 명령을 전송한다.
4. 종료 전 각 모터를 정지한 뒤 서버 터미널에서 `Ctrl+C`를 누른다.

| 제어 | 버튼 | 동작 |
| --- | --- | --- |
| DXL | `Current Mode` | 토크 OFF → 전류 모드 설정 → 토크 ON |
| DXL | `Apply` | 입력한 목표 전류 전송, 단위 A |
| DXL | `Stop` | 목표 전류 0 A |
| DXL | `Torque Off` | 토크 해제 |
| AK | `Apply ERPM` | 목표 ERPM 전송 |
| AK | `Stop ERPM` | 목표 ERPM 0 |

### 3-2. Tension / Contraction UI

**로봇 그림에서 다리를 선택해 두 모터를 함께 제어할 때 사용한다.** 통합 하드웨어 UI를 종료한 뒤 실행한다.

```bash
sudo -v
ros2 run teraffe_drive_control_agent robot_tension_web_ui \
  --serial-port /dev/ttyUSB0 --can-channel can0 \
  --host 127.0.0.1 --port 8080
```

<http://127.0.0.1:8080>에서 다리를 클릭하고 `Selected`에 표시된 모듈을 확인한다.

| 버튼 | AK 명령 | DXL 명령 |
| --- | --- | --- |
| `Tension` | `+5000 ERPM` | 전류 모드 설정 후 `-0.05 A` |
| `Contraction` | `-5000 ERPM` | 전류 모드 설정 후 `+0.05 A` |
| `Stop` | `0 ERPM` | `0 A` |

동작 값은 [robot_tension_web_ui.py](teraffe_drive_control_agent/robot_tension_web_ui.py)의 상수에서 변경한다. `Stop`은 선택한 모듈에 적용되므로 여러 모듈을 구동했다면 각각 정지한다.

### 공통 실행 옵션

| 옵션 | 기본값 | 용도 |
| --- | --- | --- |
| `--host` | `127.0.0.1` | 서버 바인딩 주소 |
| `--port` | `8080` | HTTP 포트 |
| `--serial-port` | `/dev/ttyUSB0` | DXL·엔코더 공용 포트 |
| `--dxl-baudrate` | `115200` | DXL baudrate |
| `--encoder-baudrate` | `115200` | 엔코더 baudrate |
| `--can-channel` | `can0` | SocketCAN 채널 |
| `--can-bitrate` | `1000000` | CAN bitrate |

다른 PC에서 접속하려면 `--host 0.0.0.0`으로 실행하고 `http://<로봇-PC-IP>:8080`에 접속한다. 제어 API는 인증 없이 동작하므로 접근을 허용한 내부 네트워크에서 사용한다.

### 제어 시 확인할 사항

- **브라우저 탭을 닫아도 서버와 구동 명령은 유지된다.** 정지는 화면의 정지 버튼으로 수행한다.
- 서버 종료·연결 재설정 시 명령을 보낸 AK에는 0 ERPM, DXL에는 0 A 전송을 시도한다. 통신 단절에 대비해 하드웨어 비상 정지를 준비한다.
- 웹 UI는 엔코더 각도를 표시하며, 각도 리밋을 이용한 자동 정지는 ROS 리프트 제어 경로에서 사용한다.
- ERPM은 전기적 RPM이다. 코드의 극쌍 수 `21`, 감속비 `64` 기준으로 `출력축 RPM = ERPM / 21 / 64`다.
- DXL 전류 환산은 raw 값당 `0.00269 A` 기준이다. 실제 모델의 제어 테이블과 일치시킨다.

<details>
<summary><strong>상태 API 확인</strong></summary>

```bash
curl -s http://127.0.0.1:8080/api/config | python3 -m json.tool
curl -s http://127.0.0.1:8080/api/status | python3 -m json.tool
```

`/api/config`는 연결 설정과 시리얼 포트 목록을 반환한다. `/api/status`는 `message`와 모듈별 `ak`, `dxl`, `encoder` 상태를 반환한다. 장치 응답이 2초 이상 갱신되지 않으면 해당 장치의 `connected`가 `false`가 된다. 모듈 전체의 연결 표시는 세 장치가 모두 응답할 때 활성화된다.

</details>

## 4. 데스크톱 UI 실행

모든 명령은 GUI 세션에서 실행한다. 웹 UI와 같은 시리얼 포트를 동시에 사용하지 않는다. `--no-autostart`를 지원하는 UI에서는 창을 먼저 열고 설정을 확인한 뒤 연결할 수 있다.

### 통합 하드웨어 UI

```bash
ros2 run teraffe_drive_control_agent robot_hardware_status_ui \
  --serial-port /dev/ttyUSB0 \
  --dxl-baudrate 115200 --encoder-baudrate 115200 \
  --dxl-id 111 --encoder-id 0x1C \
  --can-channel can0 --can-bitrate 1000000 --ak-id 101 \
  --no-autostart
```

<details>
<summary><strong>AK80-64 CAN 모니터</strong></summary>

SocketCAN을 먼저 설정한 뒤 실행한다.

```bash
ros2 run teraffe_drive_control_agent ak80_64_status_ui \
  --channel can0 --bitrate 1000000 --motor-id 101 --show-all
```

`--show-all`을 빼면 지정한 모터만 표시한다. `--query-mit`를 추가하면 상태 질의 프레임을 전송하므로 모터의 통신 모드를 확인한 뒤 사용한다.

</details>

<details>
<summary><strong>Dynamixel 모니터</strong></summary>

```bash
ros2 run teraffe_drive_control_agent dynamixel_status_ui \
  --port /dev/ttyUSB0 --baudrate 115200 --dxl-id 111 --period 0.1
```

</details>

<details>
<summary><strong>AMT212 엔코더 모니터</strong></summary>

```bash
ros2 run teraffe_drive_control_agent encoder_status_ui \
  --port /dev/ttyUSB0 --baudrate 115200 \
  --encoder-id 0x1C --period 0.05 --read-size 8
```

</details>

<details>
<summary><strong>DXL + 엔코더 공용 포트 UI</strong></summary>

```bash
ros2 run teraffe_drive_control_agent dxl_encoder_status_ui \
  --port /dev/ttyUSB0 \
  --dxl-baudrate 115200 --encoder-baudrate 115200 \
  --dxl-id 111 --encoder-id 0x1C --no-autostart
```

</details>

각 UI의 기본 ID는 다르므로 실제 장치에 맞게 인자를 지정한다. 전체 옵션은 `ros2 run teraffe_drive_control_agent <실행 파일> --help`로 확인한다.

## 5. ROS 리프트 제어

ROS 토픽으로 다리별 신장·수축을 제어하는 방식이다. [teraffe_config.py](teraffe_drive_control_agent/config/teraffe_config.py)의 DXL·엔코더 포트와 장치 ID를 설정하고, CAN 인터페이스를 먼저 올린다.

### 실행 순서

**터미널 1 — 하드웨어 제어 노드**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teraffe_drive_control_agent robot_controller
```

**터미널 2 — 원격 제어 화면**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teraffe_drive_control_agent ui
```

제어 화면에서 다리별 신장·수축·정지와 전체 정지를 수행한다. 제어 노드는 다리별 엔코더 각도를 읽고 설정 범위를 감시한다. 기본 범위는 `10~150°`이며 기구의 실제 동작 범위에 맞게 `lift_controller.py`에서 조정한다. 엔코더 읽기에 실패하면 구동을 거부하고 정지를 요청한다.

### 토픽 인터페이스

| 토픽 | 방향 | 메시지 타입 | 데이터 |
| --- | --- | --- | --- |
| `/robot/lift_command` | 제어 노드가 구독 | `std_msgs/msg/String` | JSON 명령 |
| `/robot/lift_status` | 제어 노드가 발행 | `std_msgs/msg/String` | 연결 상태와 다리별 각도, 발행 타이머 0.1초 |

| 필드 | 값 |
| --- | --- |
| `action` | `extend`, `retract`, `stop`, `stop_all` |
| `leg` | `front_left`, `front_right`, `rear_left`, `rear_right` |
| `erpm` | 목표 ERPM, 기본 `500` |

명령 JSON 예시는 다음과 같다.

```json
{"action": "extend", "leg": "front_left", "erpm": 500}
```

```bash
# 상태 확인
ros2 topic echo /robot/lift_status

# 전체 정지. stop_all은 leg 없이 사용할 수 있다.
ros2 topic pub --once /robot/lift_command std_msgs/msg/String \
  '{data: "{\"action\": \"stop_all\"}"}'
```

상태 JSON의 `connections`는 CAN·DXL·엔코더 포트 연결 상태다. `legs`에는 다리별 각도가 들어가며 읽지 못한 값은 `null`이다.

## 6. 주행·조향 상태 모니터

외부 MD 드라이버가 발행하는 ROS 토픽을 브라우저에 표시하는 읽기 전용 모니터다. 먼저 해당 드라이버 노드를 실행한다.

```bash
cd ~/ros2_ws/src/teraffe_drive_control_agent
python3 drive_steering_status_ui.py \
  --host 127.0.0.1 --port 8082 --stale-after 1.5
```

**접속 주소: <http://127.0.0.1:8082>**

| 토픽 | 타입 | 기대하는 데이터 |
| --- | --- | --- |
| `/motor_status` | `sensor_msgs/msg/JointState` | 이름 `motor_11`, `motor_21`; 위치 rad, 속도 RPM, 전류 mA |
| `/motor_alarms` | `std_msgs/msg/Int32MultiArray` | 첫 값: Drive ID 11, 두 번째 값: Steering ID 21 알람 |

이 모니터는 `JointState.velocity`를 RPM, `effort`를 mA로 해석한다. 발행 노드가 같은 단위를 사용해야 한다. 기본 1.5초 동안 상태가 갱신되지 않으면 미연결로 표시한다.

## 7. 장치 검색과 진단

포트를 사용 중인 UI를 종료한 뒤 저장소 루트에서 실행한다.

```bash
cd ~/ros2_ws/src/teraffe_drive_control_agent

# Dynamixel ID 111~114 응답 확인
python3 scan_dxl.py --port /dev/ttyUSB0 --baudrate 115200 \
  --start-id 111 --end-id 114

# 엔코더 요청 바이트 0~255 순회
python3 scan_encoder.py --port /dev/ttyUSB0 --baudrate 115200

# 선택한 모터의 CAN 상태를 10초 동안 수신
python3 read_cubemars_status.py --channel can0 --motor-id 104 --timeout 10

# 모든 확장 CAN 프레임 수신
python3 read_cubemars_status.py --channel can0 --show-all --timeout 10
```

`read_cubemars_status.py`는 모터 명령을 전송하지 않는 수신 전용 도구다. 엔코더 검색은 요청 바이트 전체를 순회하므로 장치의 명령 체계를 확인한 진단 환경에서 수행한다.

루트의 하드웨어 테스트는 실제 모터를 구동한다. 실행 전 파일 안의 ID와 설정을 확인한다.

| 스크립트 | 동작 |
| --- | --- |
| `test_ak80_64.py` | CAN ID `104`에 `5000 ERPM` 반복 전송 |
| `test_dynamixel.py` | DXL ID `114`에 `+0.05 A` 반복 전송 |
| `test_integrated_hardware.py` | 리프트·보조 모터·엔코더 통합 통신 및 구동 |

## 8. 개발과 검증

### 프로젝트 구조

```text
teraffe_drive_control_agent/
├── package.xml / setup.py / setup.cfg  # 의존성·빌드·실행 파일 등록
├── resource/                          # ament 등록 리소스
├── teraffe_drive_control_agent/
│   ├── config/                        # 장치·통신 설정
│   ├── hardware/                      # CAN·DXL·엔코더 통신
│   ├── modules/lift_module.py          # 다리별 모터·엔코더 조합
│   ├── controllers/                    # ROS 리프트 제어
│   ├── robot_hardware_web_ui.py        # 통합 웹 UI
│   ├── robot_tension_web_ui.py         # Tension / Contraction UI
│   ├── robot_hardware_status_ui.py     # 통합 데스크톱 UI
│   ├── *_status_ui.py                  # 장치별 데스크톱 UI
│   └── ui.py                          # ROS 원격 제어 화면
├── drive_steering_status_ui.py         # 주행·조향 웹 모니터
├── scan_dxl.py / scan_encoder.py       # 장치 검색
├── read_cubemars_status.py             # CAN 수신 도구
├── test/                              # 회귀 테스트·스타일 검사
├── test_*.py                          # 실제 하드웨어 테스트
└── reference/                         # 별도 참고 코드·모델
```

### 실행 인자 확인

```bash
ros2 run teraffe_drive_control_agent robot_hardware_web_ui --help
ros2 run teraffe_drive_control_agent robot_tension_web_ui --help
ros2 run teraffe_drive_control_agent robot_hardware_status_ui --help
```

`--help`는 CAN·시리얼 장치를 열지 않고 옵션을 출력한다.

### 회귀 테스트

```bash
cd ~/ros2_ws/src/teraffe_drive_control_agent
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_runtime.py -q
```

외부 pytest 플러그인을 자동으로 불러오지 않고, 가짜 통신 객체로 엔코더 읽기 실패, 각도 리밋, 종료 시 모터 정지, 재연결, API 입력 처리를 검증한다. 실제 장치를 열거나 모터를 구동하지 않는다. 루트의 하드웨어 테스트와 구분해 위 경로를 지정한다.

### 변경 사항 반영

Python 소스는 `--symlink-install` 빌드 후 수정 사항을 반영하기 쉽다. 실행 파일 등록이나 의존성을 변경했다면 다시 빌드한다.

```bash
cd ~/ros2_ws
colcon build --packages-select teraffe_drive_control_agent --symlink-install
source install/setup.bash
```

빌드 방식의 상세 설명은 [공식 colcon 튜토리얼](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)을 참고한다. `reference/`는 별도 참고 자료이며 위 명령은 이 패키지만 빌드한다.

## 9. 문제 해결

| 증상 | 확인할 내용 |
| --- | --- |
| ROS 패키지·실행 파일을 찾지 못함 | 빌드 후 `source ~/ros2_ws/install/setup.bash` 적용 |
| Python 모듈을 찾지 못함 | `rosdep install` 수행, ROS 시스템 Python과 환경 적용 여부 확인 |
| 시리얼 권한 오류 | `dialout` 그룹 추가 후 재로그인 |
| 시리얼 열기 실패·응답 타임아웃 | 실제 포트·baudrate·장치 ID·전원·다른 프로그램의 포트 사용 확인 |
| `can0 does not exist` | SocketCAN 어댑터·드라이버·인터페이스 이름 확인 |
| CAN 설정 권한 오류 | 서버를 실행할 터미널에서 `sudo -v` 수행 |
| CAN은 열리지만 모터 응답이 없음 | `candump can0`로 ID·bitrate·배선·종단저항·모터 상태 송신 설정 확인 |
| 엔코더 값이 갱신되지 않음 | 요청 바이트·baudrate·체크섬·어댑터의 RTS 송수신 방향 확인 |
| 웹 UI에 `--` 또는 미연결 표시 | 장치별 수신 상태 확인. 수신하지 못한 필드와 2초 이상 지난 응답은 별도 표시됨 |
| 웹 포트가 사용 중임 | 기존 서버를 종료하거나 `--port` 변경. 시리얼 포트도 중복 사용하지 않도록 확인 |
| Qt display / `xcb` 오류 | 데스크톱 UI를 GUI 세션에서 실행. 화면 없는 환경에서는 웹 UI 사용 |
| 주행·조향 상태가 보이지 않음 | 외부 발행 노드·토픽 이름·메시지 타입·모터 이름·`ROS_DOMAIN_ID` 확인 |
