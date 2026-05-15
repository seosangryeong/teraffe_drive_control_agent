#!/usr/bin/env python3
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Point, TransformStamped
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes

import tf2_ros
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def v_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def v_cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]

def v_norm(a):
    return math.sqrt(v_dot(a, a))

def v_normalize(a, eps=1e-12):
    n = v_norm(a)
    if n < eps:
        return [0.0, 0.0, 0.0]
    return [a[0]/n, a[1]/n, a[2]/n]

def quat_to_rotmat(qx, qy, qz, qw):
    xx = qx*qx; yy = qy*qy; zz = qz*qz
    xy = qx*qy; xz = qx*qz; yz = qy*qz
    wx = qw*qx; wy = qw*qy; wz = qw*qz
    return [
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ]

def quat_from_rotmat(R):
    r00, r01, r02 = R[0]; r10, r11, r12 = R[1]; r20, r21, r22 = R[2]
    tr = r00 + r11 + r22
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S; qx = (r21 - r12) / S; qy = (r02 - r20) / S; qz = (r10 - r01) / S
    elif (r00 > r11) and (r00 > r22):
        S = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / S; qx = 0.25 * S; qy = (r01 + r10) / S; qz = (r02 + r20) / S
    elif r11 > r22:
        S = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / S; qx = (r01 + r10) / S; qy = 0.25 * S; qz = (r12 + r21) / S
    else:
        S = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / S; qx = (r02 + r20) / S; qy = (r12 + r21) / S; qz = 0.25 * S
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n > 1e-12: qx /= n; qy /= n; qz /= n; qw /= n
    return qx, qy, qz, qw

def rotmat_col(R, j):
    return [R[0][j], R[1][j], R[2][j]]

def build_quat_pointing_with_free_spin(prev_q, z_target_world):
    zt = v_normalize(z_target_world)
    if v_norm(zt) < 1e-9: return prev_q
    qx, qy, qz, qw = prev_q
    Rprev = quat_to_rotmat(qx, qy, qz, qw)
    x_prev = rotmat_col(Rprev, 0)
    proj = [x_prev[0] - v_dot(x_prev, zt) * zt[0], x_prev[1] - v_dot(x_prev, zt) * zt[1], x_prev[2] - v_dot(x_prev, zt) * zt[2]]
    xn = v_normalize(proj)
    if v_norm(xn) < 1e-6:
        ref = [0.0, 1.0, 0.0]
        if abs(v_dot(ref, zt)) > 0.9: ref = [0.0, 0.0, 1.0]
        proj = [ref[0] - v_dot(ref, zt) * zt[0], ref[1] - v_dot(ref, zt) * zt[1], ref[2] - v_dot(ref, zt) * zt[2]]
        xn = v_normalize(proj)
    yn = v_normalize(v_cross(zt, xn))
    xn = v_normalize(v_cross(yn, zt))
    R = [[xn[0], yn[0], zt[0]], [xn[1], yn[1], zt[1]], [xn[2], yn[2], zt[2]]]
    return quat_from_rotmat(R)

class PandaCircleYZ_FixedCenter(Node):
    def __init__(self):
        super().__init__('panda_circle_yz_fixed_center')
        self.GROUP_NAME = "panda_arm"; self.BASE_FRAME = "panda_link0"; self.EE_LINK = "panda_hand"
        self.CENTER = Point(x=0.7, y=0.00, z=0.35); self.RADIUS = 0.15; self.POINTS = 220; self.TURNS = 1; self.CLOCKWISE = True
        self.X_DRIFT = 0.0; self.MAX_STEP = 0.003; self.JUMP_THRESHOLD = 0.0; self.AVOID_COLLISIONS = False; self.MIN_FRACTION = 0.80
        self.DO_PREMOVE_HOME = True
        self.HOME_JOINTS = {"panda_joint1": 0.0, "panda_joint2": -0.785, "panda_joint3": 0.0, "panda_joint4": -2.356, "panda_joint5": 0.0, "panda_joint6": 1.571, "panda_joint7": 0.785}
        self.JOINT_TOL = 0.01; self.DO_MOVE_TO_START_POSE = True

        self.cart_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.movegroup_client = ActionClient(self, MoveGroup, 'move_action')
        self.tf_buffer = tf2_ros.Buffer(); self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_joint_state = None
        self.arm_joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        self.current_angles_deg = [0.0] * 7
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # 애니메이션/제어 스레드 분리
        threading.Thread(target=self.run_once, daemon=True).start()

    def _joint_cb(self, msg: JointState):
        self.last_joint_state = msg
        for i, name in enumerate(self.arm_joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_angles_deg[i] = math.degrees(msg.position[idx])

    def wait_for_joint_state(self, timeout_sec=2.0):
        deadline = time.time() + timeout_sec
        while self.last_joint_state is None and time.time() < deadline: time.sleep(0.05)

    def build_start_state(self) -> RobotState:
        rs = RobotState()
        if self.last_joint_state is not None: rs.joint_state = self.last_joint_state
        return rs

    def get_current_ee_pose(self, timeout_sec=2.0) -> Pose:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(self.BASE_FRAME, self.EE_LINK, rclpy.time.Time())
                p = Pose(); p.position.x = tf.transform.translation.x; p.position.y = tf.transform.translation.y; p.position.z = tf.transform.translation.z
                p.orientation = tf.transform.rotation
                return p
            except: time.sleep(0.05)
        raise RuntimeError("TF lookup 실패")

    def move_to_joint_home(self) -> bool:
        self.wait_for_joint_state()
        if self.last_joint_state is None: return False
        req = MotionPlanRequest(); req.group_name = self.GROUP_NAME; req.start_state = self.build_start_state()
        c = Constraints()
        for jn, jv in self.HOME_JOINTS.items():
            jc = JointConstraint(); jc.joint_name = jn; jc.position = float(jv); jc.tolerance_above = self.JOINT_TOL; jc.tolerance_below = self.JOINT_TOL; jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]; req.allowed_planning_time = 5.0; req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.3; req.max_acceleration_scaling_factor = 0.3
        goal = MoveGroup.Goal(); goal.request = req; goal.planning_options.plan_only = False
        send_fut = self.movegroup_client.send_goal_async(goal)
        while not send_fut.done(): time.sleep(0.1)
        gh = send_fut.result()
        if gh is None or not gh.accepted: return False
        res_fut = gh.get_result_async()
        while not res_fut.done(): time.sleep(0.1)
        res = res_fut.result()
        return res.result.error_code.val == MoveItErrorCodes.SUCCESS

    def compute_cartesian_path(self, waypoints):
        req = GetCartesianPath.Request(); req.header.frame_id = self.BASE_FRAME; req.group_name = self.GROUP_NAME; req.waypoints = waypoints; req.max_step = self.MAX_STEP; req.jump_threshold = self.JUMP_THRESHOLD
        req.avoid_collisions = self.AVOID_COLLISIONS; req.link_name = self.EE_LINK; req.start_state = self.build_start_state()
        fut = self.cart_client.call_async(req)
        while not fut.done(): time.sleep(0.1)
        return fut.result()

    def execute_trajectory(self, traj) -> bool:
        goal = ExecuteTrajectory.Goal(); goal.trajectory = traj
        send_fut = self.exec_client.send_goal_async(goal)
        while not send_fut.done(): time.sleep(0.1)
        gh = send_fut.result()
        if gh is None or not gh.accepted: return False
        res_fut = gh.get_result_async()
        while not res_fut.done(): time.sleep(0.1)
        return res_fut.result() is not None

    def cartesian_approach_to_pose(self, target: Pose) -> bool:
        cur = self.get_current_ee_pose(); waypoints = []
        prev_q = (cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w)
        for i in range(41):
            s = i / 40.0; p = Pose()
            p.position.x = cur.position.x + (target.position.x - cur.position.x) * s
            p.position.y = cur.position.y + (target.position.y - cur.position.y) * s
            p.position.z = cur.position.z + (target.position.z - cur.position.z) * s
            qx, qy, qz, qw = build_quat_pointing_with_free_spin(prev_q, [1.0, 0.0, 0.0])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            prev_q = (qx, qy, qz, qw); waypoints.append(p)
        res = self.compute_cartesian_path(waypoints)
        if res is None or getattr(res, 'fraction', 0.0) < 0.95: return False
        return self.execute_trajectory(res.solution)

    def run_once(self):
        time.sleep(2.0)
        if self.DO_PREMOVE_HOME:
            self.get_logger().info("홈 자세 이동 중...")
            self.move_to_joint_home()
        if self.DO_MOVE_TO_START_POSE:
            start_pose = Pose(); start_pose.position.x = self.CENTER.x; start_pose.position.y = self.CENTER.y + self.RADIUS; start_pose.position.z = self.CENTER.z
            cur = self.get_current_ee_pose()
            qx, qy, qz, qw = build_quat_pointing_with_free_spin((cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w), [1.0, 0.0, 0.0])
            start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w = qx, qy, qz, qw
            self.cartesian_approach_to_pose(start_pose)
        
        cur = self.get_current_ee_pose(); total = self.POINTS * self.TURNS; sgn = -1.0 if self.CLOCKWISE else 1.0
        prev_q = (cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w); waypoints = []
        for i in range(total + 1):
            t = i / total; theta = sgn * (2.0 * math.pi * self.TURNS) * t; p = Pose()
            qx, qy, qz, qw = build_quat_pointing_with_free_spin(prev_q, [1.0, 0.0, 0.0])
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            prev_q = (qx, qy, qz, qw)
            p.position.y = self.CENTER.y + self.RADIUS * math.cos(theta); p.position.z = self.CENTER.z + self.RADIUS * math.sin(theta); p.position.x = self.CENTER.x + self.X_DRIFT * t
            waypoints.append(p)
        res = self.compute_cartesian_path(waypoints)
        if res and getattr(res, 'fraction', 0.0) >= self.MIN_FRACTION:
            self.execute_trajectory(res.solution)

def main():
    rclpy.init()
    node = PandaCircleYZ_FixedCenter()
    
    fig, ax = plt.subplots(figsize=(10, 6))
    def animate(i):
        ax.clear()
        x_pos = np.arange(len(node.arm_joint_names))
        # 가동 한계치 (Franka Panda 규격)
        limits = [166, 101, 166, 176, 166, 215, 166]
        ax.bar(x_pos, limits, color='lightgray', alpha=0.5, label='Limit')
        ax.bar(x_pos, np.abs(node.current_angles_deg), color='royalblue', label='Current (abs)')
        ax.set_xticks(x_pos); ax.set_xticklabels(node.arm_joint_names, rotation=30)
        ax.set_ylabel("Angle (Degrees)"); ax.set_ylim([0, 220])
        ax.set_title("Panda Arm Joint Monitor (Real-time)")
        ax.grid(axis='y', linestyle='--', alpha=0.7)

    ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)
    
    # ROS2 스핀을 별도 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()