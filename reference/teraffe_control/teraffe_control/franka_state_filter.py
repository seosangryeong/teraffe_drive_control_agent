#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PandaJointStateFilter(Node):
    def __init__(self):
        super().__init__('panda_joint_state_filter')

        # params
        self.declare_parameter('input_topic', '/joint_states')
        self.declare_parameter('output_topic', '/isaac_joint_states')
        self.declare_parameter('prefix', 'panda_')
        self.declare_parameter('queue_size', 10)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        self.pub = self.create_publisher(JointState, output_topic, queue_size)
        self.sub = self.create_subscription(JointState, input_topic, self.cb, queue_size)

        self.get_logger().info(
            f"Filtering JointState: in='{input_topic}' -> out='{output_topic}', prefix='{self.prefix}'"
        )

    def cb(self, msg: JointState):
        # indices of panda joints
        idx = [i for i, n in enumerate(msg.name) if n.startswith(self.prefix)]
        if not idx:
            return

        out = JointState()
        out.header = msg.header  # 그대로 전달 (stamp 포함)

        out.name = [msg.name[i] for i in idx]

        # Helper: slice only if lengths match name length
        def slice_if_aligned(arr):
            if len(arr) == len(msg.name):
                return [arr[i] for i in idx]
            # some publishers leave these empty or different length
            return []

        out.position = slice_if_aligned(msg.position)
        out.velocity = slice_if_aligned(msg.velocity)
        out.effort = slice_if_aligned(msg.effort)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = PandaJointStateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
