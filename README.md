# Teraffe Drive Control Agent

Teraffe 로봇의 AK80-64 모터, Dynamixel XH430, AMT212 엔코더를 연결해 상태를 확인하고 제어하는 ROS 2 Python 패키지다. 브라우저 기반 통합 제어 화면, PyQt5 기반 장치별 모니터, 리프트 제어용 ROS 토픽 인터페이스를 포함한다.

현재 구현의 중심은 **CAN + RS-485 하드웨어를 직접 연결하는 웹 UI**다. 웹 UI는 ROS 제어 노드 없이 독립적으로 실행할 수 있다. 기존 ROS 리프트 제어 경로는 설정 import 수정이 필요한 상태이며, 아래에 해당 제약을 별도로 정리했다.

## 목차

- [주요 기능과 실행 프로그램](#주요-기능과-실행-프로그램)
- [환경과 하드웨어 구성](#환경과-하드웨어-구성)
- [설치 방법](#설치-방법)
- [통신 설정](#통신-설정)
- [웹 UI 실행](#웹-ui-실행)
- [데스크톱 UI 실행](#데스크톱-ui-실행)
- [주행·조향 ROS 상태 모니터](#주행조향-ros-상태-모니터)
- [장치 검색과 통신 진단](#장치-검색과-통신-진단)
- [ROS 리프트 제어 인터페이스](#ros-리프트-제어-인터페이스)
- [프로젝트 구조](#프로젝트-구조)
- [검증 방법](#검증-방법)
- [문제 해결](#문제-해결)
- [구현 범위와 라이선스](#구현-범위와-라이선스)

## 주요 기능과 실행 프로그램

| 프로그램 | 용도 | 실행 방식 |
| --- | --- | --- |
| `robot_hardware_web_ui` | 4개 모듈의 모터·엔코더 상태 표시, AK ERPM 및 DXL 전류 제어 | 브라우저, 기본 포트 `8080` |
| `robot_tension_web_ui` | 통합 웹 UI에 다리 선택 및 Tension / Contraction 제어 추가 | 브라우저, 기본 포트 `8080` |
| `robot_hardware_status_ui` | AK80-64 + DXL + 엔코더 통합 모니터 및 제어 | PyQt5 |
| `ak80_64_status_ui` | CAN 상태 프레임 모니터, 선택적 MIT 상태 질의 | PyQt5 |
| `dynamixel_status_ui` | DXL 위치·속도·전류·온도·오류 확인 | PyQt5 |
| `encoder_status_ui` | AMT212 각도와 수신 상태 확인 | PyQt5 |
| `dxl_encoder_status_ui` | 공용 시리얼 포트의 DXL·엔코더 모니터 및 DXL 전류 제어 | PyQt5 |
| `ui` | ROS 토픽을 사용하는 리프트 원격 제어 화면 | PyQt5, 별도 제어 노드 필요 |
| `robot_controller` | 리프트 명령 구독 및 상태 발행 | ROS 2, 현재 설정 import 수정 필요 |
| `drive_steering_status_ui.py` | 외부 ROS 노드가 발행하는 주행·조향 상태 표시 | 소스에서 직접 실행, 기본 포트 `8082` |

`ros2 run`으로 실행 가능한 이름은 [setup.py](setup.py)의 `console_scripts`에 정의되어 있다. 장치별 모니터와 통합 UI는 각각 독립 프로그램이므로, 필요한 프로그램 하나를 선택해 실행한다.

## 환경과 하드웨어 구성

### 기준 환경

현재 개발 환경은 다음과 같다. 다른 OS·ROS 배포판 조합의 동작은 별도 확인이 필요하다.

| 항목 | 기준 |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| Python | Python 3.10 |
| 빌드 | `ament_python`, `colcon` |
| CAN | Linux SocketCAN, 기본 `can0`, `1,000,000 bit/s` |
| 시리얼 | 기본 `115200 baud` |
| GUI | PyQt5, 데스크톱 UI 사용 시 디스플레이 필요 |
| 주요 Python 의존성 | `python-can`, `pyserial`, `dynamixel-sdk`, `PyQt5` |
| ROS 의존성 | `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs` |

웹 서버 자체는 Python 표준 라이브러리의 `http.server`를 사용한다. 다만 현재 웹 UI가 CAN 상태 디코더를 PyQt5 모듈에서 가져오므로, **웹 UI만 실행하더라도 PyQt5 패키지가 필요하다**. 웹 UI 실행 시 Qt 창을 띄우지는 않는다.

### 연결 구성

```text
Ubuntu PC
├── SocketCAN 어댑터 → can0 → AK80-64 모터
└── RS-485 어댑터 → /dev/ttyUSB0 → Dynamixel + AMT212 공용 버스
```

웹 UI는 **Dynamixel과 엔코더가 하나의 시리얼 포트를 공유하는 구성**을 전제로 한다. 워커 하나가 장치별 통신 속도와 송수신 방향을 전환하면서 순차적으로 읽는다. DXL과 엔코더를 서로 다른 USB 포트에 연결한 경우에는 개별 모니터를 사용하거나 통신 코드를 해당 구성에 맞게 변경해야 한다.

### 웹 UI의 모듈별 ID

아래 매핑은 [robot_hardware_web_ui.py](teraffe_drive_control_agent/robot_hardware_web_ui.py)의 `MODULES` 기준이다. Tension UI도 같은 매핑을 사용한다.

| 모듈 | 위치 | AK80-64 CAN ID | Dynamixel ID | AMT212 요청 바이트 |
| --- | --- | --- | --- | --- |
| `FL` | 전방 좌측 | `101` | `111` | `0x1C` |
| `FR` | 전방 우측 | `102` | `112` | `0x2C` |
| `RL` | 후방 좌측 | `103` | `113` | `0x3C` |
| `RR` | 후방 우측 | `104` | `114` | `0x4C` |

이 값들은 장치에 자동으로 기록되지 않는다. 실제 장치의 설정과 코드의 매핑을 일치시켜야 한다. 엔코더 값은 코드가 그대로 전송하는 요청 바이트이며, DXL ID와 같은 번호 체계로 해석하지 않는다.

`config/teraffe_config.py`의 기존 DXL ID는 `1~4`, AK 리프트 ID는 `1~4`이며 웹 UI의 매핑과 다르다. **이 설정 파일을 수정해도 웹 UI의 `MODULES`는 바뀌지 않는다.** 프로그램마다 설정을 가져오는 위치를 확인해야 한다.

## 설치 방법

아래 명령은 Bash와 `~/ros2_ws` 워크스페이스를 기준으로 한다.

### 1. ROS 2 Humble 설치

이미 `/opt/ros/humble/setup.bash`가 있다면 다음 단계로 이동한다. 신규 Ubuntu 22.04 환경에서는 UTF-8 로케일과 ROS apt 저장소를 먼저 설정한다. 설치 절차의 기준은 [ROS 2 Humble 공식 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)다.

```bash
sudo apt update
sudo apt install -y locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```

ROS 공식 저장소 설정 패키지를 내려받아 설치한다.

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base
source /opt/ros/humble/setup.bash
```

`ros-humble-ros-base`로 필요한 ROS 런타임을 설치할 수 있다. RViz 등 ROS 데스크톱 도구도 필요하면 `ros-humble-desktop`을 선택한다. 위 저장소 패키지 명령의 `jammy`는 Ubuntu 22.04 기준이다.

### 2. 빌드 도구와 의존성 설치

```bash
sudo apt install -y \
  git python3-pip python3-setuptools \
  python3-colcon-common-extensions python3-rosdep \
  python3-pyqt5 python3-serial python3-pytest \
  ros-humble-rclpy ros-humble-std-msgs \
  ros-humble-geometry-msgs ros-humble-sensor-msgs \
  can-utils iproute2

python3 -m pip install --user python-can dynamixel-sdk
```

ROS에서 사용하는 시스템 Python과 동일한 `python3`로 설치한다. 별도 Conda 환경이나 ROS 패키지를 볼 수 없는 가상환경에서는 `rclpy` 또는 메시지 모듈 import가 실패할 수 있다.

현재 `setup.py`의 `install_requires`에는 `setuptools`만 선언되어 있다. `package.xml`에도 CAN·시리얼·Qt·DXL 런타임 의존성이 모두 선언되어 있지 않으므로, **`rosdep install`만으로 전체 실행 환경이 설치되지는 않는다.** 위 의존성 설치가 필요하다.

### 3. 저장소 복제

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/seosangryeong/teraffe_drive_control_agent.git
```

이미 복제한 저장소에서 작업 중이라면 재복제하지 않고 기존 경로를 사용한다.

### 4. ROS 의존성 설치

`rosdep`을 처음 사용하는 시스템에서만 초기화한다.

```bash
sudo rosdep init
rosdep update
```

이미 초기화되었다는 메시지가 나오면 `rosdep update`부터 수행한다. 이후 대상 패키지의 의존성을 설치한다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src/teraffe_drive_control_agent --ignore-src -r -y --rosdistro humble
```

### 5. 빌드 및 환경 적용

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select teraffe_drive_control_agent --symlink-install
source install/setup.bash
ros2 pkg executables teraffe_drive_control_agent
```

출력에 `robot_hardware_web_ui`, `robot_tension_web_ui` 등의 실행 파일이 나타나면 패키지 등록이 완료된 상태다. 빌드 방식에 관한 상세 설명은 [공식 colcon 튜토리얼](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)을 참고한다.

새 터미널마다 아래 환경을 적용한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

`--symlink-install`을 사용하면 Python 소스 수정 사항을 반영하기 쉽다. `setup.py`의 실행 파일 등록이나 패키지 메타데이터를 변경했다면 다시 빌드한다.

## 통신 설정

### 1. 시리얼 포트 확인 및 권한 설정

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$USER"
```

`dialout` 그룹 추가 후에는 로그아웃하고 다시 로그인해야 권한이 적용된다. 실제 포트를 확인해 실행 인자의 `/dev/ttyUSB0`를 바꾼다. USB 연결 순서에 따라 번호가 달라지면 `/dev/serial/by-id/`의 고정 경로를 사용할 수 있다.

같은 시리얼 포트를 여는 프로그램은 한 번에 하나만 실행한다. 통합 웹 UI, 개별 DXL·엔코더 모니터, 장치 검색 스크립트를 동시에 실행하면 응답 충돌이나 타임아웃이 발생할 수 있다.

### 2. SocketCAN 설정

SocketCAN을 지원하는 어댑터를 연결하고 인터페이스 이름을 확인한다.

```bash
ip -brief link
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

`can0`가 존재해야 하며 실제 모터의 CAN bitrate도 같아야 한다. 어댑터가 `/dev/ttyUSB*`로만 보이는 경우에는 해당 어댑터의 드라이버·동작 모드 설정을 먼저 확인한다. 이 패키지는 시리얼 CAN 어댑터를 자동으로 SocketCAN으로 변환하지 않는다.

수신 프레임은 다음 명령으로 확인한다. 종료는 `Ctrl+C`다.

```bash
candump can0
```

웹 UI는 시작 및 `Apply Connections` 실행 시 `ip link`로 CAN 인터페이스를 내렸다가 bitrate를 설정하고 다시 올린다. 일반 사용자로 실행하면 내부에서 `sudo`를 사용하므로, **웹 UI를 시작할 터미널에서 먼저 `sudo -v`를 실행한다.** 이미 CAN을 올렸더라도 웹 UI가 재설정한다. 다른 프로그램과 CAN을 공유 중이라면 이 재설정으로 통신이 잠시 끊길 수 있다.

### 3. 실행 전 확인

실제 구동 전에는 장치를 고정하고 모듈별 ID·회전 방향·전류 단위가 하드웨어와 일치하는지 확인한다. 웹 UI는 모터 명령을 직접 전송하며, 엔코더 값에 따른 자동 정지나 브라우저 연결 해제 시 자동 정지를 구현하지 않는다. 정지 버튼과 하드웨어 비상 정지를 사용할 수 있는 상태에서 구동한다.

## 웹 UI 실행

### 통합 하드웨어 웹 UI

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
sudo -v
ros2 run teraffe_drive_control_agent robot_hardware_web_ui \
  --host 127.0.0.1 --port 8080 \
  --serial-port /dev/ttyUSB0 \
  --dxl-baudrate 115200 --encoder-baudrate 115200 \
  --can-channel can0 --can-bitrate 1000000
```

브라우저에서 <http://127.0.0.1:8080>에 접속한다. ROS 토픽 제어 노드를 별도로 띄울 필요는 없다.

1. 연결 설정에서 시리얼 포트, DXL·엔코더 baudrate, CAN 채널을 확인한다.
2. 설정을 변경했다면 `Apply Connections`를 눌러 통신 워커를 재시작한다.
3. FL / FR / RL / RR의 AK·DXL·엔코더 상태를 각각 확인한다. 세 장치 모두 최근 응답이 있어야 모듈 전체가 연결 상태로 표시된다.
4. DXL 제어 대상 모듈을 선택하고 `Current Mode`로 전류 제어 모드를 설정한다. 이 동작은 토크를 끄고 모드를 바꾼 뒤 토크를 다시 켠다.
5. 전류값을 A 단위로 입력하고 `Apply`로 전송한다. `Stop`은 목표 전류를 0으로 만들며, `Torque Off`는 토크를 해제한다.
6. AK 제어 대상과 ERPM을 선택하고 `Apply ERPM`으로 전송한다. `Stop ERPM`은 목표 ERPM을 0으로 만든다.
7. 종료 전 AK와 DXL을 각각 정지하고, 필요하면 DXL 토크를 해제한 뒤 서버 터미널에서 `Ctrl+C`를 누른다.

AK 목표 ERPM은 서버가 약 0.1초 간격으로 재전송한다. 브라우저 탭을 닫아도 서버와 명령 재전송은 계속된다. 서버 종료 시 AK는 저장된 대상에 0 ERPM 전송을 시도하지만, **DXL 목표 전류를 자동으로 0으로 만드는 종료 처리는 없다.** 종료 전에 화면에서 정지해야 한다.

ERPM은 모터의 전기적 RPM이다. 코드의 극쌍 수 `21`, 감속비 `64` 기준 환산은 `출력축 RPM = ERPM / 21 / 64`다. 실제 모터 설정이 다르면 환산값도 달라진다. 웹 UI의 DXL 전류 환산은 raw 값당 `0.00269 A`를 사용하므로, 사용하는 모델의 제어 테이블과 일치하는지 확인한다.

### Tension / Contraction 웹 UI

기본 포트와 하드웨어 포트를 공유하므로 통합 하드웨어 웹 UI를 먼저 종료한다.

```bash
sudo -v
ros2 run teraffe_drive_control_agent robot_tension_web_ui \
  --host 127.0.0.1 --port 8080 \
  --serial-port /dev/ttyUSB0 --can-channel can0
```

<http://127.0.0.1:8080>에서 로봇 그림의 다리를 클릭해 하나의 모듈을 선택한다. `Selected`에 표시된 모듈에 다음 명령을 전송한다.

| 버튼 | AK 명령 | DXL 명령 |
| --- | --- | --- |
| `Tension` | `+5000 ERPM` | 전류 모드 설정 후 `-0.05 A` |
| `Contraction` | `-5000 ERPM` | 전류 모드 설정 후 `+0.05 A` |
| `Stop` | `0 ERPM` | 목표 전류 `0 A` |

값은 [robot_tension_web_ui.py](teraffe_drive_control_agent/robot_tension_web_ui.py)의 상수에 정의되어 있다. 두 모터의 명령은 별도 워커의 큐로 전달되므로 동시에 성공하는 원자적 동작은 아니다. 버튼을 놓거나 브라우저를 닫는 동작은 정지 명령이 아니며, `Stop`은 현재 선택한 모듈에만 적용된다. 여러 모듈을 구동했다면 각각 정지한다.

### 웹 UI 공통 옵션

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--host` | `127.0.0.1` | HTTP 서버 바인딩 주소 |
| `--port` | `8080` | HTTP 포트 |
| `--serial-port` | `/dev/ttyUSB0` | DXL·엔코더 공용 포트 |
| `--dxl-baudrate` | `115200` | DXL 통신 속도 |
| `--encoder-baudrate` | `115200` | 엔코더 통신 속도 |
| `--can-channel` | `can0` | SocketCAN 인터페이스 |
| `--can-bitrate` | `1000000` | CAN 통신 속도 |

같은 네트워크의 다른 PC에서 접근하려면 `--host 0.0.0.0`으로 실행하고 `http://<로봇-PC-IP>:8080`으로 접속한다. 제어 API에 인증이 없으므로 접근을 허용한 내부 네트워크에서 사용한다.

### 상태 API

```bash
curl -s http://127.0.0.1:8080/api/config | python3 -m json.tool
curl -s http://127.0.0.1:8080/api/status | python3 -m json.tool
```

`/api/config`는 사용 가능한 시리얼 포트와 현재 연결 설정을 반환한다. `/api/status`는 `message`, `modules`를 반환하며 각 모듈 아래에 `ak`, `dxl`, `encoder` 상태가 들어간다. 최근 수신 후 2초가 지나면 해당 장치의 `connected`가 `false`로 표시된다. 연결 표시가 바뀌어도 모터를 자동으로 정지시키지는 않는다.

## 데스크톱 UI 실행

아래 예시는 웹 UI와 별도로 실행한다. `--no-autostart`를 사용하면 창을 먼저 열고 설정을 확인한 뒤 연결을 시작할 수 있다.

### AK80-64 CAN 모니터

```bash
ros2 run teraffe_drive_control_agent ak80_64_status_ui \
  --channel can0 --bitrate 1000000 --motor-id 101 --show-all
```

`--show-all`을 빼면 지정한 모터만 표시한다. 기본 동작은 상태 프레임 수신이며, `--query-mit`를 추가하면 MIT 상태 질의 프레임을 주기적으로 전송한다. 질의 옵션은 모터의 통신 모드를 확인한 뒤 사용한다. 단독 CAN 모니터는 실행 전에 SocketCAN을 직접 설정해야 한다.

### Dynamixel 모니터

```bash
ros2 run teraffe_drive_control_agent dynamixel_status_ui \
  --port /dev/ttyUSB0 --baudrate 115200 --dxl-id 111 --period 0.1
```

### AMT212 엔코더 모니터

```bash
ros2 run teraffe_drive_control_agent encoder_status_ui \
  --port /dev/ttyUSB0 --baudrate 115200 \
  --encoder-id 0x1C --period 0.05 --read-size 8
```

### DXL + 엔코더 공용 포트 UI

```bash
ros2 run teraffe_drive_control_agent dxl_encoder_status_ui \
  --port /dev/ttyUSB0 \
  --dxl-baudrate 115200 --encoder-baudrate 115200 \
  --dxl-id 111 --encoder-id 0x1C --no-autostart
```

### AK + DXL + 엔코더 통합 UI

```bash
ros2 run teraffe_drive_control_agent robot_hardware_status_ui \
  --serial-port /dev/ttyUSB0 \
  --dxl-baudrate 115200 --encoder-baudrate 115200 \
  --dxl-id 111 --encoder-id 0x1C \
  --can-channel can0 --can-bitrate 1000000 --ak-id 101 \
  --no-autostart
```

데스크톱 UI의 기본 ID는 프로그램마다 다르므로 위 예시처럼 인자를 명시하는 편이 좋다. 각 프로그램의 전체 옵션은 `ros2 run teraffe_drive_control_agent <프로그램명> --help`로 확인한다.

## 주행·조향 ROS 상태 모니터

[drive_steering_status_ui.py](drive_steering_status_ui.py)는 외부 MD 모터 드라이버의 ROS 토픽을 구독하는 읽기 전용 웹 모니터다. 이 저장소가 해당 드라이버 노드까지 실행해 주지는 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/teraffe_drive_control_agent
python3 drive_steering_status_ui.py --host 127.0.0.1 --port 8082 --stale-after 1.5
```

브라우저에서 <http://127.0.0.1:8082>에 접속한다. 이 스크립트는 `console_scripts`에 등록되어 있지 않아 `python3`로 실행한다.

| 토픽 | 메시지 타입 | 코드가 기대하는 데이터 |
| --- | --- | --- |
| `/motor_status` | `sensor_msgs/msg/JointState` | `name`: `motor_11`, `motor_21`; `position`: rad; `velocity`: RPM; `effort`: mA |
| `/motor_alarms` | `std_msgs/msg/Int32MultiArray` | 첫 번째 값: Drive ID 11, 두 번째 값: Steering ID 21 알람 |

위 `velocity`와 `effort`의 해석은 이 모니터의 구현 기준이다. 외부 발행 노드가 같은 단위를 사용하는지 확인한다. 기본 1.5초 동안 상태가 갱신되지 않으면 연결이 끊긴 것으로 표시한다.

## 장치 검색과 통신 진단

아래 스크립트는 저장소 루트에서 실행한다. 포트를 사용 중인 UI를 먼저 종료한다.

```bash
cd ~/ros2_ws/src/teraffe_drive_control_agent

# Dynamixel ID 111~114 응답 확인
python3 scan_dxl.py --port /dev/ttyUSB0 --baudrate 115200 --start-id 111 --end-id 114

# AMT212 요청 바이트 0~255 순회
python3 scan_encoder.py --port /dev/ttyUSB0 --baudrate 115200

# CubeMars/VESC 확장 CAN 상태 프레임을 10초 동안 수신
python3 read_cubemars_status.py --channel can0 --motor-id 104 --timeout 10

# 모든 확장 CAN 프레임 확인
python3 read_cubemars_status.py --channel can0 --show-all --timeout 10
```

`read_cubemars_status.py`는 모터 명령을 보내지 않는 수신 전용 스크립트다. 모터에서 상태 프레임을 송신하지 않으면 화면에 값이 나타나지 않는다. `scan_encoder.py`는 모든 요청 바이트를 전송하므로 엔코더의 명령 체계를 확인한 진단 환경에서 사용한다.

루트의 `test_ak80_64.py`, `test_dynamixel.py`, `test_integrated_hardware.py` 등은 실제 하드웨어 구동·통신용 스크립트다. 예를 들어 현재 `test_ak80_64.py`는 CAN ID `104`에 `5000 ERPM`을 반복 전송하고, `test_dynamixel.py`는 DXL ID `114`에 `+0.05 A`를 반복 전송한다. 파일 안의 안내 문구와 실제 코드가 다를 수 있으므로, 실행 전 ID·명령·종료 처리를 코드에서 확인한다.

## ROS 리프트 제어 인터페이스

### 현재 실행 제약

`robot_controller`는 현재 소스 그대로 실행하면 `ImportError`가 발생한다. [lift_controller.py](teraffe_drive_control_agent/controllers/lift_controller.py)가 다음 상수를 실제 정의 위치와 다른 파일에서 가져오기 때문이다.

| 상수 | 현재 import 위치 | 실제 정의 위치 |
| --- | --- | --- |
| `DEVICENAME`, `BAUDRATE` | `config/dynamixel_config.py` | `config/teraffe_config.py`에 각각 `DYNAMIXEL_PORT`, `DYNAMIXEL_BAUDRATE`로 정의 |
| `DYNAMIXEL_IDS` | `config/dynamixel_config.py` | `config/teraffe_config.py` |
| `AK80_64_IDS` | `config/ak80_64_config.py` | `config/teraffe_config.py` |

이 경로를 사용하려면 import를 정리하고, 실제 장치 ID·포트·엔코더 주소를 일치시켜야 한다. `LiftModule`의 기본 엔코더 주소는 `0x54`이며 현재 컨트롤러는 다리별 주소를 따로 넘기지 않는다. 기존 컨트롤러는 DXL과 엔코더를 별도로 여는 구조이므로 웹 UI의 공용 포트 구성과도 차이가 있다.

아래는 **해당 설정을 수정한 뒤 사용하는 실행 절차와 인터페이스 명세**다.

### 실행 순서

첫 번째 터미널에서 하드웨어를 준비하고 제어 노드를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teraffe_drive_control_agent robot_controller
```

두 번째 터미널에서 같은 ROS 환경을 적용한 뒤 원격 UI를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teraffe_drive_control_agent ui
```

`ui`만 실행해도 창은 열 수 있지만 실제 제어와 상태 수신에는 `robot_controller`가 필요하다. 내부적으로 `lift_controller`와 `robot_controller` 두 ROS 노드를 실행한다.

### 토픽과 명령 형식

| 토픽 | 방향 | 타입 | 내용 |
| --- | --- | --- | --- |
| `/robot/lift_command` | 제어 노드가 구독 | `std_msgs/msg/String` | JSON 명령 |
| `/robot/lift_status` | 제어 노드가 발행 | `std_msgs/msg/String` | 연결 상태와 다리별 엔코더 각도, 타이머 주기 0.1초 |

명령의 `action`은 `extend`, `retract`, `stop`, `stop_all`이다. `leg`는 `front_left`, `front_right`, `rear_left`, `rear_right` 중 하나이며 `erpm`의 기본값은 `500`이다. `stop_all`은 `leg` 없이 사용할 수 있다.

```json
{"action": "extend", "leg": "front_left", "erpm": 500}
```

```bash
# 상태 수신
ros2 topic echo /robot/lift_status

# 전체 정지 명령
ros2 topic pub --once /robot/lift_command std_msgs/msg/String \
  '{data: "{\"action\": \"stop_all\"}"}'
```

상태 JSON의 `connections`에는 통신 연결 상태, `legs`에는 다리별 각도 또는 읽기 실패 시 `null`이 들어간다. 컨트롤러의 각도 리밋 `10~150°`는 테스트용 값이며, 이 로직은 웹 UI 제어 경로에 적용되지 않는다.

## 프로젝트 구조

```text
teraffe_drive_control_agent/
├── README.md
├── package.xml                       # ROS 의존성 및 패키지 정보
├── setup.py                          # Python 패키지와 실행 파일 등록
├── setup.cfg
├── resource/                         # ament 패키지 등록 리소스
├── teraffe_drive_control_agent/
│   ├── config/                       # 통신·모터·엔코더 설정 상수
│   ├── hardware/                     # CAN, DXL, AK, 엔코더 통신
│   ├── modules/lift_module.py         # 리프트 + 보조 모터 + 엔코더 조합
│   ├── controllers/                  # ROS 리프트·통합 제어 노드
│   ├── ros_nodes/                    # 확장용 빈 파일
│   ├── robot_hardware_web_ui.py       # 4개 모듈 웹 상태·직접 제어
│   ├── robot_tension_web_ui.py        # 다리 Tension / Contraction UI
│   ├── robot_hardware_status_ui.py    # PyQt 통합 UI
│   ├── ak80_64_status_ui.py           # CAN 모니터 및 상태 디코더
│   ├── dynamixel_status_ui.py         # DXL 모니터
│   ├── encoder_status_ui.py           # 엔코더 모니터
│   ├── dxl_encoder_status_ui.py       # 공용 시리얼 UI
│   └── ui.py                         # ROS 리프트 원격 UI
├── drive_steering_status_ui.py        # 외부 주행·조향 토픽 웹 모니터
├── scan_dxl.py / scan_encoder.py      # 시리얼 장치 응답 검색
├── read_cubemars_status.py            # CAN 상태 수신 전용 도구
├── test_*.py / scan_*.py              # 하드웨어 구동·통신 진단 스크립트
├── test/                             # ament 코드 스타일 검사
└── reference/teraffe_control/         # 별도 참고 코드·모델
```

`reference/`에는 별도 ROS 패키지와 비전·로봇 모델 관련 코드가 들어 있다. 위 설치와 빌드는 `teraffe_drive_control_agent`만 대상으로 하며 참고 코드의 모든 의존성까지 설치하지 않는다.

## 검증 방법

### 장치를 열지 않는 실행 인자 확인

```bash
ros2 run teraffe_drive_control_agent robot_hardware_web_ui --help
ros2 run teraffe_drive_control_agent robot_tension_web_ui --help
ros2 run teraffe_drive_control_agent robot_hardware_status_ui --help
```

`--help`는 실제 CAN·시리얼 통신을 시작하기 전에 종료된다. 이 확인은 import와 CLI 진입만 검증하며 실제 장치 동작을 보장하지 않는다.

### 코드 스타일 검사

```bash
cd ~/ros2_ws
colcon test --packages-select teraffe_drive_control_agent
colcon test-result --verbose
```

`test/`에는 flake8·pep257·copyright용 ament 검사가 있다. 현재 copyright 검사는 skip 상태이며 기존 코드의 스타일 문제로 나머지 검사가 실패할 수 있다. 이 검사들은 하드웨어 기능 테스트가 아니다.

저장소 루트에서 `pytest` 전체 탐색을 실행하면 실제 통신용 `test_*.py`까지 수집할 수 있으므로, 스타일 검사만 직접 실행할 때는 `python3 -m pytest test/`처럼 범위를 지정한다.

## 문제 해결

| 증상 | 확인 및 조치 |
| --- | --- |
| `Package 'teraffe_drive_control_agent' not found` | 워크스페이스에서 빌드한 뒤 `source ~/ros2_ws/install/setup.bash`를 실행한다. |
| `No executable found` | `ros2 pkg executables teraffe_drive_control_agent`로 등록 이름을 확인하고 `setup.py` 변경 후 다시 빌드한다. 루트의 진단 스크립트는 `python3`로 실행한다. |
| `No module named can / serial / dynamixel_sdk / PyQt5` | 의존성을 같은 Python에 설치한다. 웹 UI에도 PyQt5가 필요하다. |
| `No module named rclpy` | `/opt/ros/humble/setup.bash`를 적용하고 시스템 Python을 사용한다. |
| `cannot import name 'DEVICENAME'` | 기존 ROS 제어 노드의 설정 import 불일치다. 위 ROS 리프트 제어 절을 참고한다. |
| `Permission denied: /dev/ttyUSB*` | `dialout` 그룹과 재로그인 여부, 장치 파일 권한을 확인한다. |
| `Serial open failed` | 실제 포트 경로, USB 연결, 다른 프로그램의 포트 사용 여부를 확인한다. |
| `can0 does not exist` | 어댑터의 SocketCAN 지원, 드라이버, 실제 인터페이스 이름을 확인한다. |
| `CAN setup failed` 또는 sudo 인증 요구 | 실행 터미널에서 `sudo -v` 후 시작한다. 웹 UI가 시작·재연결 시 CAN 설정 명령을 실행한다. |
| CAN은 열리지만 모터가 미연결로 표시됨 | `candump can0`로 수신 여부, 모터 ID, bitrate, 배선·종단저항·상태 송신 설정을 확인한다. 포트가 열린 것과 모터 응답은 별개다. |
| DXL 응답 타임아웃 | Protocol 2.0, 실제 baudrate·ID, 전원, RS-485 배선을 확인한다. 웹 UI의 기본 ID는 `111~114`다. |
| 엔코더가 미연결로 표시됨 | 요청 바이트, baudrate, 체크섬, RS-485 RTS 송수신 방향을 확인한다. 어댑터에 따라 방향 제어 방식이 다를 수 있다. |
| 일부 값이 `--`로 표시됨 | 해당 상태 패킷이 없거나 읽기에 실패한 경우다. 전체 통신 상태와 개별 필드 오류를 함께 확인한다. |
| `Address already in use` | 기존 웹 서버를 종료하거나 `--port 8081` 등으로 변경한다. 포트만 바꿔도 시리얼 포트 공유 충돌은 해결되지 않는다. |
| Qt `xcb` / display 오류 | 데스크톱 UI는 GUI 세션에서 실행한다. 화면 없는 환경에서는 웹 UI를 사용한다. |
| 주행·조향 모니터에 값이 없음 | 외부 발행 노드, 토픽 이름·타입, `motor_11`·`motor_21`, 같은 `ROS_DOMAIN_ID` 사용 여부를 확인한다. |

## 구현 범위와 라이선스

- `controllers/drive_controller.py`, `controllers/safety_controller.py`, `hardware/steering_motor.py`, `modules/wheel_module.py`, `main.py`, `ros_nodes/`의 노드 파일은 현재 비어 있다. 완성된 주행·조향 제어기나 독립 안전 제어기로 동작하지 않는다.
- 현재 실행 설정은 주로 Python 상수와 CLI 인자로 관리한다. 전용 launch 파일이나 YAML 기반 통합 설정은 포함되어 있지 않다.
- `package.xml`과 `setup.py`의 버전은 `0.0.0`이며 설명·라이선스 필드는 TODO 상태다. 저장소 루트에 프로젝트 라이선스 파일이 없으므로 공개 배포 시 적용할 라이선스를 별도로 정해야 한다. `reference/` 코드와 외부 SDK의 라이선스는 각각 확인해야 한다.
