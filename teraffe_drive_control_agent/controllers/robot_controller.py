import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json

from teraffe_drive_control_agent.controllers.lift_controller import LiftController

class RobotControllerNode(Node):
    """
    통합 로봇 제어 노드
    - 하드웨어 컨트롤러(LiftController 등)를 관리합니다.
    - 외부 UI나 원격 제어기로부터 ROS 2 명령을 받고(Subscribe), 상태를 전송(Publish)합니다.
    """
    def __init__(self, lift_controller):
        super().__init__('robot_controller')
        self.lift_node = lift_controller
        
        # 상태 전송 퍼블리셔
        self.status_pub = self.create_publisher(String, '/robot/lift_status', 10)
        
        # 제어 명령 수신 서브스크라이버
        self.cmd_sub = self.create_subscription(String, '/robot/lift_command', self.cmd_callback, 10)
        
        # 0.1초마다 상태를 JSON으로 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info("Robot Controller Pub/Sub 노드가 시작되었습니다.")

    def cmd_callback(self, msg):
        """UI 등에서 보낸 JSON 형태의 제어 명령을 파싱하여 하드웨어 제어 메서드 호출"""
        try:
            cmd_data = json.loads(msg.data)
            action = cmd_data.get('action')
            leg = cmd_data.get('leg')
            erpm = cmd_data.get('erpm', 500)
            
            self.get_logger().info(f"명령 수신: {action} on {leg}")
            
            if action == 'extend':
                self.lift_node.extend_leg(leg, erpm)
            elif action == 'retract':
                self.lift_node.retract_leg(leg, erpm)
            elif action == 'stop':
                module = self.lift_node.get_lift(leg)
                if module:
                    module.stop()
            elif action == 'stop_all':
                self.lift_node.stop_all()
        except Exception as e:
            self.get_logger().error(f"명령 처리 중 오류 발생: {e}")

    def publish_status(self):
        """현재 하드웨어 상태를 읽어와 JSON 형태로 퍼블리시"""
        status_dict = {
            "connections": self.lift_node.get_connection_status(),
            "legs": {}
        }
        
        leg_names = ["front_left", "front_right", "rear_left", "rear_right"]
        for name in leg_names:
            status_dict["legs"][name] = self.lift_node.get_leg_position(name)
            
        msg = String()
        msg.data = json.dumps(status_dict)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # 1. 하드웨어 컨트롤러 노드 생성 (기존 LiftController)
    lift_node = LiftController()
    
    # 2. 통합 제어 퍼블리셔/서브스크라이버 노드 생성
    robot_node = RobotControllerNode(lift_node)
    
    # 3. 두 노드를 동시에 실행하기 위해 MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(lift_node)
    executor.add_node(robot_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_node.get_logger().info("Shutting down Robot Controller...")
        lift_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
