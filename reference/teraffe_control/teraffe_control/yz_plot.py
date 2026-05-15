#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class RealTimeTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('realtime_trajectory_plotter')
        
        # TF 리스너 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 데이터 저장 공간 (최근 500개의 점만 유지)
        self.max_points = 500
        self.y_data = deque(maxlen=self.max_points)
        self.z_data = deque(maxlen=self.max_points)

        # 그래프 설정
        plt.ion() # 대화형 모드 On
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.line, = self.ax.plot([], [], 'ro-', markersize=2, label='Path')
        self.ax.set_xlabel('Y Coordinate (m)')
        self.ax.set_ylabel('Z Coordinate (m)')
        self.ax.set_title('End-Effector Trajectory (YZ Plane)')
        self.ax.grid(True)
        self.ax.axis('equal') # 비율 1:1 강제
        self.ax.legend()

        # 0.05초마다 TF를 확인하는 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.update_plot)
        self.get_logger().info("📈 실시간 궤적 모니터링 시작...")

    def update_plot(self):
        try:
            # World 기준 panda_hand의 현재 위치 가져오기
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('World', 'panda_hand', now)
            
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z

            # 데이터 추가
            self.y_data.append(curr_y)
            self.z_data.append(curr_z)

            # 그래프 업데이트
            self.line.set_data(self.y_data, self.z_data)
            
            # 축 범위 자동 조절 (원의 중심에 맞춰서)
            if len(self.y_data) > 0:
                padding = 0.06
                self.ax.set_xlim(min(self.y_data) - padding, max(self.y_data) + padding)
                self.ax.set_ylim(min(self.z_data) - padding, max(self.z_data) + padding)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        except Exception as e:
            # TF를 아직 못 찾았을 때는 무시
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeTrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()