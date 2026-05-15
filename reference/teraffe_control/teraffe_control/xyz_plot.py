#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import numpy as np

class RealTimeTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('realtime_trajectory_plotter')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.max_points = 3000
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        self.z_data = deque(maxlen=self.max_points)

        self.start_pos = None
        self.start_marker = None

        plt.ion()
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line, = self.ax.plot([], [], [], 'ro-', markersize=2, label='Path', linewidth=1)
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('3D Trajectory of End-Effector')
        self.ax.grid(True)
        
        self.last_pos = None
        self.jump_threshold = 0.5

        self.timer = self.create_timer(0.05, self.update_plot)

    def update_plot(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('World', 'panda_hand', now, rclpy.duration.Duration(seconds=0.01))
            
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            curr_z = trans.transform.translation.z

            if self.last_pos is not None:
                dist = np.sqrt((curr_x - self.last_pos[0])**2 + (curr_y - self.last_pos[1])**2 + (curr_z - self.last_pos[2])**2)
                if dist > self.jump_threshold:
                    return

            if self.start_pos is None:
                self.start_pos = (curr_x, curr_y, curr_z)
                self.start_marker = self.ax.scatter([curr_x], [curr_y], [curr_z], color='blue', marker='*', s=150, label='Start', zorder=5)
                self.ax.legend()

            self.x_data.append(curr_x)
            self.y_data.append(curr_y)
            self.z_data.append(curr_z)
            self.last_pos = (curr_x, curr_y, curr_z)

            self.line.set_data(list(self.x_data), list(self.y_data))
            self.line.set_3d_properties(list(self.z_data))
            
            if len(self.x_data) > 1:
                padding = 0.1
                self.ax.set_xlim(min(list(self.x_data) + [self.start_pos[0]]) - padding, max(list(self.x_data) + [self.start_pos[0]]) + padding)
                self.ax.set_ylim(min(list(self.y_data) + [self.start_pos[1]]) - padding, max(list(self.y_data) + [self.start_pos[1]]) + padding)
                self.ax.set_zlim(min(list(self.z_data) + [self.start_pos[2]]) - padding, max(list(self.z_data) + [self.start_pos[2]]) + padding)

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception:
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()