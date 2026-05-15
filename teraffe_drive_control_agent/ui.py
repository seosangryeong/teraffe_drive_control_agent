import sys
import threading
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QGroupBox, QGridLayout
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject

class UI_ROS_Node(Node):
    """원격 조종기 역할을 하는 ROS 2 Node (GUI 백그라운드에서 동작)"""
    def __init__(self, signal_emitter):
        super().__init__('robot_gui_node')
        self.signal_emitter = signal_emitter
        
        # Publisher: 사용자가 버튼을 누르면 로봇으로 명령(JSON) 전송
        self.cmd_pub = self.create_publisher(String, '/robot/lift_command', 10)
        
        # Subscriber: 로봇에서 보내는 상태(JSON) 수신
        self.status_sub = self.create_subscription(String, '/robot/lift_status', self.status_callback, 10)
        
        self.get_logger().info("UI ROS 2 Node Started. Waiting for robot status...")
        
    def status_callback(self, msg):
        """로봇 컨트롤러로부터 상태 메시지를 수신하여 PyQt GUI 갱신 시그널 발생"""
        try:
            status_data = json.loads(msg.data)
            # ROS 스레드에서 직접 GUI 컴포넌트를 조작하면 크래시가 나므로 Signal을 통해 전달
            self.signal_emitter.status_received.emit(status_data)
        except Exception as e:
            self.get_logger().error(f"Status Parse Error: {e}")
            
    def send_command(self, action, leg=None, erpm=500):
        """JSON 형태로 명령 포맷을 만들어 퍼블리시"""
        cmd_dict = {
            "action": action,
            "leg": leg,
            "erpm": erpm
        }
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.cmd_pub.publish(msg)

class SignalEmitter(QObject):
    # PyQt Signal: ROS 2 백그라운드 스레드에서 GUI(메인 스레드)로 데이터를 안전하게 전달
    status_received = pyqtSignal(dict)

class RobotGUI(QMainWindow):
    """Teraffe Remote Control Panel GUI"""
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.erpm_target = 500
        
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Teraffe Remote Control Panel (ROS 2)")
        self.resize(700, 350)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        
        # ==========================================
        # 하드웨어 연결 상태 그룹 (원격 모니터링)
        # ==========================================
        status_group = QGroupBox("Hardware Connections (Remote Monitoring)")
        status_layout = QHBoxLayout()
        
        self.conn_labels = {}
        for hw_name in ["CAN", "DXL", "ENC"]:
            lbl = QLabel(hw_name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: gray; color: white; padding: 5px; border-radius: 5px;")
            self.conn_labels[hw_name] = lbl
            status_layout.addWidget(lbl)
            
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # ==========================================
        # Lift 제어 그룹
        # ==========================================
        lift_group = QGroupBox("Lift Control (다리 리프트 제어)")
        lift_layout = QGridLayout()
        
        self.leg_names = ["front_left", "front_right", "rear_left", "rear_right"]
        self.status_labels = {}
        
        for i, name in enumerate(self.leg_names):
            name_label = QLabel(name.replace("_", " ").upper())
            name_label.setStyleSheet("font-weight: bold; font-size: 14px;")
            
            status_label = QLabel("Angle: -- °")
            status_label.setStyleSheet("font-size: 14px;")
            self.status_labels[name] = status_label
            
            btn_extend = QPushButton("Extend (상승)")
            btn_extend.setStyleSheet("background-color: #8fce00; color: white; padding: 5px;")
            btn_extend.clicked.connect(lambda checked, n=name: self.on_extend(n))
            
            btn_retract = QPushButton("Retract (하강)")
            btn_retract.setStyleSheet("background-color: #ff9900; color: white; padding: 5px;")
            btn_retract.clicked.connect(lambda checked, n=name: self.on_retract(n))
            
            btn_stop = QPushButton("Stop")
            btn_stop.setStyleSheet("background-color: #ff4c4c; color: white; padding: 5px;")
            btn_stop.clicked.connect(lambda checked, n=name: self.on_stop(n))
            
            lift_layout.addWidget(name_label, i, 0)
            lift_layout.addWidget(status_label, i, 1)
            lift_layout.addWidget(btn_extend, i, 2)
            lift_layout.addWidget(btn_retract, i, 3)
            lift_layout.addWidget(btn_stop, i, 4)
            
        lift_group.setLayout(lift_layout)
        main_layout.addWidget(lift_group)
        
        # ==========================================
        # 전역 제어 그룹 (Global Control)
        # ==========================================
        global_group = QGroupBox("Global Control")
        global_layout = QHBoxLayout()
        
        btn_stop_all = QPushButton("🛑 STOP ALL (전체 비상 정지 명령 퍼블리시)")
        btn_stop_all.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; font-size: 16px; padding: 10px;")
        btn_stop_all.clicked.connect(self.on_stop_all)
        
        global_layout.addWidget(btn_stop_all)
        global_group.setLayout(global_layout)
        
        main_layout.addWidget(global_group)
        main_layout.addStretch()
        
        central_widget.setLayout(main_layout)

    # UI 버튼 이벤트 처리 -> ROS 노드를 통해 퍼블리시
    def on_extend(self, leg_name):
        self.ros_node.send_command(action="extend", leg=leg_name, erpm=self.erpm_target)

    def on_retract(self, leg_name):
        self.ros_node.send_command(action="retract", leg=leg_name, erpm=self.erpm_target)

    def on_stop(self, leg_name):
        self.ros_node.send_command(action="stop", leg=leg_name)

    def on_stop_all(self):
        self.ros_node.send_command(action="stop_all")

    # ROS 노드로부터 데이터(Signal) 수신 시 실행
    def update_status(self, status_data):
        # 1. 연결 상태 갱신
        conns = status_data.get("connections", {})
        for hw_name, is_connected in conns.items():
            if hw_name in self.conn_labels:
                lbl = self.conn_labels[hw_name]
                if is_connected:
                    lbl.setText(f"{hw_name}: OK")
                    lbl.setStyleSheet("background-color: #8fce00; color: white; padding: 5px; border-radius: 5px; font-weight: bold;")
                else:
                    lbl.setText(f"{hw_name}: ERR")
                    lbl.setStyleSheet("background-color: #ff4c4c; color: white; padding: 5px; border-radius: 5px; font-weight: bold;")
                    
        # 2. 다리 각도 갱신
        legs = status_data.get("legs", {})
        for name, deg in legs.items():
            if name in self.status_labels:
                if deg is not None:
                    self.status_labels[name].setText(f"Angle: {deg:.2f}°")
                else:
                    self.status_labels[name].setText("Angle: Err/Disconn")

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    
    # 1. PyQt5 App 생성
    app = QApplication(sys.argv)
    
    # 2. Signal Emitter 생성
    emitter = SignalEmitter()
    
    # 3. UI 통신용 ROS 2 노드 생성
    ros_node = UI_ROS_Node(emitter)
    
    # 4. GUI 인스턴스화
    gui = RobotGUI(ros_node)
    
    # 5. Signal 연결 (ROS Subscriber -> GUI Update)
    emitter.status_received.connect(gui.update_status)
    gui.show()
    
    # 6. ROS 스레드 시작
    spin_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    # 7. GUI 실행 (사용자가 닫을 때까지 대기)
    exit_code = app.exec_()
    
    # 8. 종료 처리
    ros_node.get_logger().info("Shutting down UI ROS Node...")
    ros_node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
