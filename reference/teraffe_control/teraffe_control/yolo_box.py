#!/usr/bin/env python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
from rclpy.node import Node


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')

        self.bridge = CvBridge()
        self.model = YOLO('/home/nuc/valve_yolo/runs/detect/train3/weights/best.pt')
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            # '/rgb',
            self.image_callback,
            10
        )

        self.result_pub = self.create_publisher(Image, '/yolo/result_image', 10)

        self.get_logger().info('realsense_yolo_node started')
        self.get_logger().info('model loaded')
        self.get_logger().info('subscribed: /camera/camera/color/image_raw')
        self.get_logger().info('publishing: /yolo/result_image')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)
        annotated = results[0].plot()

        result_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        result_msg.header = msg.header
        self.result_pub.publish(result_msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'processed {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()