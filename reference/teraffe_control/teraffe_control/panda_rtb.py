#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import roboticstoolbox as rtb
from spatialmath import SE3

import numpy as np
import math
import threading
import time

import tf2_ros
from geometry_msgs.msg import TransformStamped


def _quat_to_R(x, y, z, w):
    q = np.array([w, x, y, z], dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=float)
    return R


def _tf_to_SE3(t: TransformStamped):
    tr = t.transform.translation
    qr = t.transform.rotation
    R = _quat_to_R(qr.x, qr.y, qr.z, qr.w)
    p = np.array([tr.x, tr.y, tr.z], dtype=float)
    return SE3.Rt(R, p)


class PandaGripClosedNode(Node):
    def __init__(self):
        super().__init__('panda_grip_closed_node')

        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        self.panda = rtb.models.DH.Panda()

        self.arm_joint_names = [f"panda_joint{i+1}" for i in range(7)]
        self.finger_joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        self.all_joint_names = self.arm_joint_names + self.finger_joint_names

        self.q_actual = np.zeros(7)

        self.center = [0.712, -0.01, 0.502]
        self.radius = 0.08

        self.gripper_closed_pos = [0.0, 0.0]

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.base_frame = "World"
        self.hand_frame = "panda_hand"

        self.T_ee_to_hand = None

        threading.Thread(target=self.run_loop, daemon=True).start()

    def _joint_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for i, jn in enumerate(self.arm_joint_names):
            if jn in name_to_idx and len(msg.position) > name_to_idx[jn]:
                self.q_actual[i] = msg.position[name_to_idx[jn]]

    def _lookup_T_base_hand(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.hand_frame, rclpy.time.Time())
            return _tf_to_SE3(tf)
        except Exception:
            return None

    def _calibrate_ee_to_hand(self):
        T_base_hand = self._lookup_T_base_hand()
        if T_base_hand is None:
            return False

        T_base_ee = self.panda.fkine(self.q_actual)
        self.T_ee_to_hand = (T_base_ee.inv() * T_base_hand)
        return True

    def _make_R_hand_z_to_xbase_keep_current_roll(self, R_hand_current):
        z_des = np.array([1.0, 0.0, 0.0], dtype=float)

        x_cur = R_hand_current[:, 0]
        x_proj = x_cur - np.dot(x_cur, z_des) * z_des
        if np.linalg.norm(x_proj) < 1e-8:
            fallback = np.array([0.0, 1.0, 0.0], dtype=float)
            x_proj = fallback - np.dot(fallback, z_des) * z_des

        x_des = x_proj / np.linalg.norm(x_proj)
        y_des = np.cross(z_des, x_des)
        y_des = y_des / np.linalg.norm(y_des)

        R_des = np.column_stack((x_des, y_des, z_des))
        return R_des

    def run_loop(self):
        time.sleep(2.0)
        self.get_logger().info("시작")

        while rclpy.ok() and (self.T_ee_to_hand is None):
            ok = self._calibrate_ee_to_hand()
            if ok:
                self.get_logger().info("EE->HAND 오프셋 캘리브레이션 완료")
                break
            self.get_logger().info("TF 대기중(panda_link0 -> panda_hand)")
            time.sleep(0.2)

        if self.T_ee_to_hand is None:
            self.get_logger().error("캘리브레이션 실패: TF를 못 읽어서 종료")
            return

        while rclpy.ok():
            for i in range(180):
                theta = (2 * math.pi / 180) * i

                x = self.center[0]
                y = self.center[1] + self.radius * math.cos(theta)
                z = self.center[2] + self.radius * math.sin(theta)

                T_base_hand_now = self._lookup_T_base_hand()
                if T_base_hand_now is None:
                    time.sleep(0.02)
                    continue

                R_hand_now = np.array(T_base_hand_now.R)
                R_hand_des = self._make_R_hand_z_to_xbase_keep_current_roll(R_hand_now)

                T_target_hand = SE3.Rt(R_hand_des, [x, y, z])
                T_target_ee = T_target_hand * self.T_ee_to_hand.inv()

                sol = self.panda.ik_LM(T_target_ee, q0=self.q_actual, mask=[1, 1, 1, 1, 1, 0])

                if sol[1]:
                    cmd = JointState()
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    cmd.name = self.all_joint_names
                    cmd.position = sol[0].tolist() + self.gripper_closed_pos
                    self.joint_pub.publish(cmd)

                time.sleep(0.04)


def main(args=None):
    rclpy.init(args=args)
    node = PandaGripClosedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()