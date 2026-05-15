import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
import math

class TeraffeControlPublisher(Node):
    def __init__(self):
        super().__init__('teraffe_control_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        #  로봇 규격(늘어나지않았을때) (1,3:좌 / 2,4:우) 
        self.L = 1.0   
        self.W = 0.74  
        
        # --- 상태 변수 ---
        self.up_axis_val = 1.0           
        self.down_axis_val = 1.0         
        self.current_total_height = 0.0  
        self.move_speed = 0.02           
        self.prismatic_deadzone = 0.1    

        # 틸트 관련
        self.front_tilt = 0.0
        self.back_tilt = 0.0
        self.tilt_speed = 0.02
        self.btn_front_down = 0 
        self.btn_back_down = 0  

        # Steer 관련
        self.steer_axis_val = 0.0        # Axis 3 (Crab)
        self.turn_axis_val = 0.0         # Axis 0 (Ackermann)
        self.pivot_axis_val = 0.0        # Axis 6 (Pivot)
        self.current_steer_angle = 0.0   
        self.steer_limit = 0.7      
        self.steer_smoothing = 0.4       

        # Drive 관련
        self.drive_axis_val = 0.0        # Axis 1 (Drive)
        self.max_drive_velocity = 10.0   

        self.joint_names = [
            'j1_1', 'j2_1', 'j3_1', 'j4_1',
            'j1_2', 'j2_2', 'j3_2', 'j4_2',
            'j1_steer', 'j2_steer', 'j3_steer', 'j4_steer',
            'j1_drive', 'j2_drive', 'j3_drive', 'j4_drive'
        ]

    def joy_callback(self, msg):
        if len(msg.axes) > 6: self.pivot_axis_val = msg.axes[6]
        if len(msg.axes) > 5: self.up_axis_val = msg.axes[5]
        if len(msg.axes) > 3: self.steer_axis_val = -msg.axes[3]
        if len(msg.axes) > 2: self.down_axis_val = msg.axes[2]
        if len(msg.axes) > 1: self.drive_axis_val = msg.axes[1]
        if len(msg.axes) > 0: self.turn_axis_val = msg.axes[0]   
        
        if len(msg.buttons) > 5:
            self.btn_front_down = msg.buttons[4]
            self.btn_back_down = msg.buttons[5]
        
        # B 버튼 누를 때 초기화
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            self.current_steer_angle = 0.0
            self.front_tilt = 0.0
            self.back_tilt = 0.0

    def timer_callback(self):
        # 1. Prismatic & Tilt 높이 계산
        up_norm = (1.0 - self.up_axis_val) / 2.0
        down_norm = (1.0 - self.down_axis_val) / 2.0
        prismatic_move = up_norm if up_norm > self.prismatic_deadzone else (-down_norm if down_norm > self.prismatic_deadzone else 0.0)
        self.current_total_height += prismatic_move * self.move_speed
        self.current_total_height = max(0.0, min(self.current_total_height, 1.6))

        if self.btn_front_down: self.front_tilt = min(self.front_tilt + self.tilt_speed, 0.8)
        if self.btn_back_down: self.back_tilt = min(self.back_tilt + self.tilt_speed, 0.8)

        height_front = max(0.0, self.current_total_height - self.front_tilt)
        height_back = max(0.0, self.current_total_height - self.back_tilt)
        f_tier1, f_tier2 = min(height_front, 0.8), max(0.0, height_front - 0.8)
        b_tier1, b_tier2 = min(height_back, 0.8), max(0.0, height_back - 0.8)

        # 2. 통합 제어 로직
        steer_vals = [0.0] * 4
        drive_vels = [0.0] * 4  
        rad_53 = 53.0 * (math.pi / 180.0) / 2

        # 드라이브 명령
        normal_cmd = self.drive_axis_val * self.max_drive_velocity

        # --- 모드 결정 로직 ---
        
        # Prismatic 작동
        if abs(prismatic_move) > 0.01:
            steer_vals[0], steer_vals[1] = rad_53, -rad_53
            steer_vals[2], steer_vals[3] = -rad_53, rad_53
            drive_vels = [-normal_cmd, normal_cmd, -normal_cmd, normal_cmd]
            self.current_steer_angle = 0.0

        # 제자리 회전
        elif abs(self.pivot_axis_val) > 0.1:
            steer_vals[0], steer_vals[1] = -rad_53, rad_53
            steer_vals[2], steer_vals[3] = rad_53, -rad_53
            pivot_cmd = -self.pivot_axis_val * self.max_drive_velocity 
            drive_vels = [pivot_cmd, pivot_cmd, pivot_cmd, pivot_cmd]
            self.current_steer_angle = 0.0

        # Crab 조향 (Axis 3) 
        elif abs(self.steer_axis_val) > 0.1:
            # 90도까지 조향 가능
            target_steer = self.steer_axis_val * (math.pi / 2)  # 90도
            self.current_steer_angle += (target_steer - self.current_steer_angle) * self.steer_smoothing
            for i in range(4): steer_vals[i] = self.current_steer_angle
            
            drive_vels = [-normal_cmd, normal_cmd, -normal_cmd, normal_cmd]


        # Ackermann 조향 (Axis 0) 
        elif abs(self.turn_axis_val) > 0.1:
            target_steer = self.turn_axis_val * self.steer_limit
            self.current_steer_angle += (target_steer - self.current_steer_angle) * self.steer_smoothing
            
            target_delta = abs(self.current_steer_angle)
            if target_delta > 0.001:
                R = (self.L / 2) / math.tan(target_delta)
                inner_angle = math.atan((self.L / 2) / (R - self.W / 2))
                outer_angle = math.atan((self.L / 2) / (R + self.W / 2))
                if self.current_steer_angle > 0: inner_angle, outer_angle = -inner_angle, -outer_angle
            else: inner_angle = outer_angle = 0.0

            if self.current_steer_angle > 0: 
                steer_vals[0], steer_vals[1] = inner_angle, outer_angle 
                steer_vals[2], steer_vals[3] = -inner_angle, -outer_angle 
            else: 
                steer_vals[0], steer_vals[1] = outer_angle, inner_angle 
                steer_vals[2], steer_vals[3] = -outer_angle, -inner_angle 
                
            if normal_cmd >= 0:
                drive_vels = [-normal_cmd-4, normal_cmd+4, -normal_cmd-4, normal_cmd+4]
            else:
                drive_vels = [-normal_cmd+4, normal_cmd-4, -normal_cmd+4, normal_cmd-4]

        # [모드 E] 중립/주행 상태
        else:
            # 주행 스틱(Axis 1)도 조작하지 않을 때만 0도로 복구
            if abs(self.drive_axis_val) < 0.1:
                if self.current_steer_angle > 0.25: self.current_steer_angle -= 0.25
                elif self.current_steer_angle < -0.25: self.current_steer_angle += 0.25
                else: self.current_steer_angle = 0.0
            
            # 주행 중일 때는 마지막 current_steer_angle을 유지
            for i in range(4): steer_vals[i] = self.current_steer_angle
            drive_vels = [-normal_cmd, normal_cmd, -normal_cmd, normal_cmd]

        # 3. 메시지 발행
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        positions = [0.0] * 16
        positions[0], positions[1] = f_tier1, f_tier1
        positions[2], positions[3] = b_tier1, b_tier1
        positions[4], positions[5] = f_tier2, f_tier2
        positions[6], positions[7] = b_tier2, b_tier2
        for i in range(4): positions[8 + i] = steer_vals[i]
        
        velocities = [0.0] * 16
        velocities[12], velocities[13] = drive_vels[0], drive_vels[1]
        velocities[14], velocities[15] = drive_vels[2], drive_vels[3]
        
        msg.position, msg.velocity = positions, velocities
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeraffeControlPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()