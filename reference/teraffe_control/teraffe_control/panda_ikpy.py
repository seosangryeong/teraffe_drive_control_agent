#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ikpy.chain
import numpy as np
import time
import math

def wrap_to_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def make_R_from_z_and_y(z_axis, y_hint):
    z = np.array(z_axis, dtype=float)
    z = z / np.linalg.norm(z)

    y = np.array(y_hint, dtype=float)
    y = y - np.dot(y, z) * z
    yn = np.linalg.norm(y)
    if yn < 1e-9:
        y = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
        y = y - np.dot(y, z) * z
        y = y / np.linalg.norm(y)
    else:
        y = y / yn

    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    return np.stack([x, y, z], axis=1)

class PandaIkpyNode(Node):
    def __init__(self):
        super().__init__('panda_ikpy_node')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        urdf_path = "/home/nuc/ros2_ws/src/teraffe_control/panda_description/urdf/panda.urdf"
        full_chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=["panda_link0"])

        links = []
        for l in full_chain.links:
            links.append(l)
            if l.name == "panda_hand":
                break
        self.chain = ikpy.chain.Chain(links=links)

        active = {f"panda_joint{i+1}" for i in range(7)}
        self.chain.active_links_mask = [lnk.name in active for lnk in self.chain.links]

        # 원의 중심 및 반지름 설정
        self.center = np.array([0.7, 0.0, 0.5], dtype=float)
        self.radius = 0.08
        self.period = 4.0

        self.R_target = make_R_from_z_and_y([1.0, 0.0, 0.0], [0.0, 0.0, 1.0])

        self.q = np.zeros(len(self.chain.links), dtype=float)
        if len(self.q) >= 8:
            self.q[1:8] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        self.ik_seed = np.zeros(len(self.chain.links), dtype=float)
        if len(self.ik_seed) >= 8:
            self.ik_seed[1:8] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        self.joint_names = [f"panda_joint{i+1}" for i in range(7)]
        self.all_joint_names = self.joint_names + ["panda_finger_joint1", "panda_finger_joint2"]

        self.dt = 1.0 / 200.0
        self.kp_pos = 6.0
        self.kp_ori = 8.0

        self.vmax = np.array([1.8, 1.8, 1.8, 2.2, 2.2, 2.6, 2.6], dtype=float)
        self.alpha = 0.20

        self.initialized = False
        self.t0 = time.monotonic()
        self.timer = self.create_timer(self.dt, self.step)

    def joint_callback(self, msg):
        if not self.initialized:
            try:
                for i, name in enumerate(self.joint_names):
                    idx = msg.name.index(name)
                    self.q[i+1] = msg.position[idx]
                self.initialized = True
                self.t0 = time.monotonic()
            except ValueError:
                pass

    def step(self):
        if not self.initialized:
            return

        if not hasattr(self, 'p_init'):
            self.p_init = self.chain.forward_kinematics(self.q)[:3, 3]

        t_total = time.monotonic() - self.t0
        
        # 접근 단계를 위한 시간 설정
        warmup_phase1 = 2.5  # 1단계: 안전 지점까지 YZ 이동 시간
        warmup_phase2 = 1.5  # 2단계: 밸브 진입 (X축 이동) 시간
        warmup_total = warmup_phase1 + warmup_phase2

        # 원 궤적의 시작점
        p_circle_start = self.center + np.array([0.0, self.radius, 0.0], dtype=float)
        
        # 1단계 목표: 밸브 앞 10cm 떨어진 안전한 정렬 지점
        p_safe_approach = self.center + np.array([-0.1, 0.0, 0.0], dtype=float)

        if t_total < warmup_phase1:
            # 1단계: 현재 위치에서 안전 정렬 지점까지 YZ 평면 위주로 부드럽게 이동
            ratio = 0.5 * (1.0 - math.cos(math.pi * (t_total / warmup_phase1)))
            p_des = self.p_init + (p_safe_approach - self.p_init) * ratio
            
        elif t_total < warmup_total:
            # 2단계: 안전 정렬 지점에서 원의 시작점까지 X축 방향으로만 직선 진입
            t_phase2 = t_total - warmup_phase1
            ratio = 0.5 * (1.0 - math.cos(math.pi * (t_phase2 / warmup_phase2)))
            p_des = p_safe_approach + (p_circle_start - p_safe_approach) * ratio

        else:
            # 3단계: 원형 궤적 운동 시작
            t = t_total - warmup_total
            theta = 2.0 * math.pi * ((t / self.period) % 1.0)
            p_des = self.center + np.array([0.0,
                                            self.radius * math.cos(theta),
                                            self.radius * math.sin(theta)], dtype=float)

        try:
            full_q = self.chain.inverse_kinematics(
                target_position=p_des.tolist(),
                target_orientation=self.R_target,
                orientation_mode="all",
                initial_position=self.ik_seed
            )
            self.ik_seed = full_q.copy()
        except Exception:
            full_q = self.ik_seed

        q_sol = full_q[1:8].copy()

        q_prev = self.q[1:8].copy()
        q_sol = q_prev + wrap_to_pi(q_sol - q_prev)

        dq = q_sol - q_prev
        max_dq = self.vmax * self.dt
        
        exceed_ratio = np.max(np.abs(dq) / max_dq)
        if exceed_ratio > 1.0:
            dq = dq / exceed_ratio
            
        q_limited = q_prev + dq

        q_cmd = q_prev + self.alpha * (q_limited - q_prev)

        self.q = full_q.copy()
        self.q[1:8] = wrap_to_pi(q_cmd)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names
        msg.position = self.q[1:8].tolist() + [0.0, 0.0]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PandaIkpyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()