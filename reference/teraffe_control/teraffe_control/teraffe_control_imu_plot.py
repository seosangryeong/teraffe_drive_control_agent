import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from tf2_ros import TransformListener, Buffer
import math
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# 쿼터니안 -> 오일러 
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class TeraffeIMUControlPublisher(Node):
    def __init__(self):
        super().__init__('teraffe_control_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_period = 0.02 
        self.timer = self.create_timer(self.control_period, self.timer_callback)
        self.start_time = time.time()
        
        self.history_t = []
        self.history_roll = []
        self.history_pitch = []
        self.history_mode = []
        
        self.log_dir = os.path.join(os.path.dirname(__file__), 'logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        self.L, self.W = 1.0, 0.74
        self.up_axis_val, self.down_axis_val = 1.0, 1.0
        self.drive_axis_val = 0.0
        self.current_total_height, self.move_speed = 0.8, 0.02
        self.prismatic_deadzone = 0.1
        self.auto_level_mode, self.prev_btn_a = False, 0
        self.estop_mode, self.prev_btn_y = False, 0 
        self.dir_pitch, self.dir_roll = 1.0, 1.0
        
        # self.kp_pitch, self.kd_pitch = 0.06, 0.5
        self.kp_pitch, self.kd_pitch = 0.06, 0.5
        self.kp_roll = self.kp_pitch * (self.W / self.L)
        self.kd_roll = self.kd_pitch * (self.W / self.L)
        
        self.prev_pitch_error, self.prev_roll_error = 0.0, 0.0
        self.leg_extension = [0.0, 0.0, 0.0, 0.0]
        self.front_tilt, self.back_tilt, self.tilt_speed = 0.0, 0.0, 0.02
        self.btn_front_down, self.btn_back_down = 0, 0
        self.steer_axis_val, self.turn_axis_val, self.pivot_axis_val = 0.0, 0.0, 0.0
        self.current_steer_angle, self.steer_limit, self.steer_smoothing = 0.0, 0.7, 0.4
        self.max_drive_velocity = 10.0
        
        self.joint_names = [
            'j1_1', 'j2_1', 'j3_1', 'j4_1',
            'j1_2', 'j2_2', 'j3_2', 'j4_2',
            'j1_steer', 'j2_steer', 'j3_steer', 'j4_steer',
            'j1_drive', 'j2_drive', 'j3_drive', 'j4_drive'
        ]
        self.actual_positions = {name: 0.0 for name in self.joint_names}

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.actual_positions:
                self.actual_positions[name] = pos

    def joy_callback(self, msg):
        # 3번 버튼(Y) 누르면 전체 초기화
        if len(msg.buttons) > 3:
            current_btn_y = msg.buttons[3]
            if current_btn_y == 1 and self.prev_btn_y == 0:
                self.current_total_height = 0.8
                self.current_steer_angle = 0.0
                self.front_tilt = 0.0
                self.back_tilt = 0.0
                self.leg_extension = [0.0, 0.0, 0.0, 0.0]
                self.prev_pitch_error = self.prev_roll_error = 0.0
                self.auto_level_mode = False
                print("RESET")
            self.prev_btn_y = current_btn_y

        # 제자리 회전
        if len(msg.axes) > 6: self.pivot_axis_val = msg.axes[6]
        # 다리 신장  
        if len(msg.axes) > 5: self.up_axis_val = msg.axes[5]
        # 조향각 제어
        if len(msg.axes) > 3: self.steer_axis_val = -msg.axes[3]
        # 다리 축소
        if len(msg.axes) > 2: self.down_axis_val = msg.axes[2]
        # 전후진 속도 제어
        if len(msg.axes) > 1: self.drive_axis_val = msg.axes[1]
        # 회전 제어
        if len(msg.axes) > 0: self.turn_axis_val = msg.axes[0]
        
        # 앞쪽 다리 기울이기, 뒤쪽 다리 기울이기
        if len(msg.buttons) > 5:
            self.btn_front_down, self.btn_back_down = msg.buttons[4], msg.buttons[5]
            
        # A버튼 누르면 오토레벨링 
        if len(msg.buttons) > 0:
            current_btn_a = msg.buttons[0]
            if current_btn_a == 1 and self.prev_btn_a == 0:
                self.auto_level_mode = not self.auto_level_mode
                if not self.auto_level_mode: 
                    self.leg_extension = [0.0, 0.0, 0.0, 0.0]
                    self.prev_pitch_error = self.prev_roll_error = 0.0
                print(f"AUTO LEVEL: {self.auto_level_mode}")
            self.prev_btn_a = current_btn_a

    # panda 베이스를 기준으로 쿼터니안 측정
    def get_orientation_error(self):
        try:
            t = self.tf_buffer.lookup_transform('World', 'panda_link0', rclpy.time.Time())
            q = t.transform.rotation
            return euler_from_quaternion(q.x, q.y, q.z, q.w)[:2]
        except Exception: return 0.0, 0.0


    def timer_callback(self):
        # panda 베이스의 롤, 피치
        curr_roll, curr_pitch = self.get_orientation_error()
        
        self.history_t.append(time.time() - self.start_time)
        self.history_roll.append(math.degrees(curr_roll))
        self.history_pitch.append(math.degrees(curr_pitch))
        self.history_mode.append(1.0 if self.auto_level_mode else 0.0)
        
        actual_h = [
            self.actual_positions.get('j1_1', 0.0) + self.actual_positions.get('j1_2', 0.0),
            self.actual_positions.get('j2_1', 0.0) + self.actual_positions.get('j2_2', 0.0),
            self.actual_positions.get('j3_1', 0.0) + self.actual_positions.get('j3_2', 0.0),
            self.actual_positions.get('j4_1', 0.0) + self.actual_positions.get('j4_2', 0.0)
        ]

        # 다리 신장/축소 
        # 안누르면 0, 완전히 누르면 -1/1 (축소/신장) 선형적으로 증감
        up_norm = (1.0 - self.up_axis_val) / 2.0
        down_norm = (1.0 - self.down_axis_val) / 2.0
        # 데드존(0.1) 이상일 때만 반영
        prismatic_move = up_norm if up_norm > self.prismatic_deadzone else (-down_norm if down_norm > self.prismatic_deadzone else 0.0)
        # 새 높이 = 현재 높이 + (이동량 × 속도)
        self.current_total_height = max(0.0, min(self.current_total_height + prismatic_move * self.move_speed, 1.6))
        # 수동 조작 감지 : 다리 신장/축소 입력 or 앞/뒤 다리 기울이기 버튼 눌리면 오토레벨링 일시중지
        is_manual_active = (abs(prismatic_move) > 0.01) or (self.btn_front_down == 1) or (self.btn_back_down == 1)
        
        if is_manual_active:
            self.leg_extension = [0.0, 0.0, 0.0, 0.0]
            self.prev_pitch_error, self.prev_roll_error = curr_pitch, curr_roll
        
        # 앞부분 기울기(최대 0.8)
        if self.btn_front_down: 
            self.front_tilt = min(self.front_tilt + self.tilt_speed, 0.8)
            self.back_tilt = max(0.0, self.back_tilt - self.tilt_speed)

        #  뒷부분 기울기(최대 0.8)
        if self.btn_back_down: 
            self.back_tilt = min(self.back_tilt + self.tilt_speed, 0.8)
            self.front_tilt = max(0.0, self.front_tilt - self.tilt_speed)

        if self.auto_level_mode and not is_manual_active:
            d_p = (curr_pitch - self.prev_pitch_error)
            d_r = (curr_roll - self.prev_roll_error)
            p_s = (curr_pitch * self.kp_pitch + d_p * self.kd_pitch) * self.dir_pitch
            r_s = (curr_roll * self.kp_roll + d_r * self.kd_roll) * self.dir_roll
            
            deltas = [(p_s + r_s), (p_s - r_s), (-p_s + r_s), (-p_s - r_s)]
            
            min_ext, max_ext = -1.6, 1.6
            for i in range(4):
                desired_pos = self.leg_extension[i] + deltas[i]
                residual = 0.0
                if desired_pos > max_ext:
                    residual = desired_pos - max_ext
                    self.leg_extension[i] = max_ext
                elif desired_pos < min_ext:
                    residual = desired_pos - min_ext
                    self.leg_extension[i] = min_ext
                else:
                    self.leg_extension[i] = desired_pos
                
                if abs(residual) > 0.0001:
                    opposite_idx = 3 - i
                    self.leg_extension[opposite_idx] = max(min_ext, min(self.leg_extension[opposite_idx] - residual, max_ext))
            
            self.prev_pitch_error, self.prev_roll_error = curr_pitch, curr_roll
        elif not self.auto_level_mode:
            self.prev_pitch_error, self.prev_roll_error = curr_pitch, curr_roll

        h = [max(0.0, min(self.current_total_height - self.front_tilt + self.leg_extension[0], 1.6)),
             max(0.0, min(self.current_total_height - self.front_tilt + self.leg_extension[1], 1.6)),
             max(0.0, min(self.current_total_height - self.back_tilt + self.leg_extension[2], 1.6)),
             max(0.0, min(self.current_total_height - self.back_tilt + self.leg_extension[3], 1.6))]

        def calc_tiers(val):
            v = max(0.0, min(val, 1.6))
            return min(v, 0.8), max(0.0, v - 0.8)

        t1, t2 = zip(*(calc_tiers(v) for v in h))
        
        steer_vals, drive_vels = [0.0]*4, [0.0]*4
        rad_53, normal_cmd = 53.0 * (math.pi / 180.0) / 2, self.drive_axis_val * self.max_drive_velocity

        if abs(prismatic_move) > 0.01:
            steer_vals, drive_vels = [rad_53, -rad_53, -rad_53, rad_53], [-normal_cmd, normal_cmd, -normal_cmd, normal_cmd]
        elif abs(self.pivot_axis_val) > 0.1:
            steer_vals = [-rad_53, rad_53, rad_53, -rad_53]
            drive_vels = [-self.pivot_axis_val * self.max_drive_velocity]*4
        elif abs(self.steer_axis_val) > 0.1:
            self.current_steer_angle += (self.steer_axis_val * (math.pi / 2) - self.current_steer_angle) * self.steer_smoothing
            steer_vals, drive_vels = [self.current_steer_angle]*4, [-normal_cmd, normal_cmd, -normal_cmd, normal_cmd]
        elif abs(self.turn_axis_val) > 0.1:
            self.current_steer_angle += (self.turn_axis_val * self.steer_limit - self.current_steer_angle) * self.steer_smoothing
            td = abs(self.current_steer_angle)
            if td > 0.001:
                R = (self.L/2)/math.tan(td)
                ia, oa = math.atan((self.L/2)/(R-self.W/2)), math.atan((self.L/2)/(R+self.W/2))
                if self.current_steer_angle > 0: ia, oa = -ia, -oa
                steer_vals = [ia, oa, -ia, -oa] if self.current_steer_angle > 0 else [oa, ia, -oa, -ia]
            drive_vels = [-normal_cmd-2, normal_cmd+2, -normal_cmd-2, normal_cmd+2]
        else:
            if abs(self.drive_axis_val) < 0.1:
                self.current_steer_angle *= 0.9
            steer_vals, drive_vels = [self.current_steer_angle]*4, [-normal_cmd, normal_cmd, -normal_cmd, normal_cmd]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(t1) + list(t2) + steer_vals + [0.0]*4
        msg.velocity = [0.0]*12 + drive_vels
        self.publisher_.publish(msg)

    def generate_report(self):
        if not self.history_t: return
        plt.figure(figsize=(12, 10))
        plt.subplot(3, 1, 1)
        plt.plot(self.history_t, self.history_roll)
        plt.grid(True)
        plt.ylabel('Roll (deg)')
        plt.subplot(3, 1, 2)
        plt.plot(self.history_t, self.history_pitch)
        plt.grid(True)
        plt.ylabel('Pitch (deg)')
        plt.subplot(3, 1, 3)
        plt.plot(self.history_t, self.history_mode)
        plt.grid(True)
        plt.ylabel('Auto Level Mode')
        plt.xlabel('Time (s)')
        plt.ylim(-0.1, 1.1)
        log_file = os.path.join(self.log_dir, 'robot_performance_plot.png')
        plt.savefig(log_file)
        plt.close()
        print(f"Plot saved to {log_file}")


def main(args=None):
    rclpy.init(args=args)
    node = TeraffeIMUControlPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.generate_report()
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()