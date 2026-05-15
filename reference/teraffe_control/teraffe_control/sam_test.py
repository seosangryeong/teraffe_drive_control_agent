import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch

from segment_anything import sam_model_registry, SamPredictor


class SamRealSenseTest(Node):
    def __init__(self):
        super().__init__('sam_realsense_test')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('sam_checkpoint', '/home/nuc/sam/sam_vit_l_0b3195.pth')
        self.declare_parameter('model_type', 'vit_l')

        self.image_topic = self.get_parameter('image_topic').value
        self.sam_checkpoint = self.get_parameter('sam_checkpoint').value
        self.model_type = self.get_parameter('model_type').value

        self.bridge = CvBridge()
        self.latest_image = None
        self.display_image = None
        self.clicked_points = []
        self.mask_overlay = None

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        sam = sam_model_registry[self.model_type](checkpoint=self.sam_checkpoint)
        sam.to(device=self.device)
        self.predictor = SamPredictor(sam)

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        cv2.namedWindow('SAM RealSense')
        cv2.setMouseCallback('SAM RealSense', self.mouse_callback)

        self.timer = self.create_timer(0.03, self.show_image)

        self.get_logger().info(f'image_topic: {self.image_topic}')
        self.get_logger().info(f'sam_checkpoint: {self.sam_checkpoint}')
        self.get_logger().info(f'model_type: {self.model_type}')
        self.get_logger().info(f'device: {self.device}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = frame.copy()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.clicked_points) < 3:
                self.clicked_points.append((x, y))
                self.mask_overlay = None

    def run_sam_once(self):
        if self.latest_image is None:
            return

        if len(self.clicked_points) == 0:
            self.get_logger().warn('먼저 화면에서 분할할 위치를 클릭하세요.')
            return

        image_bgr = self.latest_image.copy()
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        self.predictor.set_image(image_rgb)

        input_point = np.array(self.clicked_points)
        input_label = np.ones(len(self.clicked_points), dtype=np.int32)

        masks, scores, logits = self.predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=True
        )

        best_idx = int(np.argmax(scores))
        mask = masks[best_idx].astype(np.uint8)

        overlay = image_bgr.copy()
        overlay[mask > 0] = (0.5 * overlay[mask > 0] + 0.5 * np.array([0, 255, 0])).astype(np.uint8)

        contours, _ = cv2.findContours(mask * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (0, 255, 255), 2)

        for x, y in self.clicked_points:
            cv2.circle(overlay, (x, y), 5, (0, 0, 255), -1)

        self.mask_overlay = overlay
        self.get_logger().info(f'SAM score: {scores[best_idx]:.4f}')

    def show_image(self):
        if self.latest_image is None:
            return

        if self.mask_overlay is not None:
            show = self.mask_overlay.copy()
        else:
            show = self.latest_image.copy()

        for x, y in self.clicked_points:
            cv2.circle(show, (x, y), 5, (0, 0, 255), -1)

        cv2.putText(show, 'Click target point, press s: run SAM, r: reset, q: quit',
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('SAM RealSense', show)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.run_sam_once()

        elif key == ord('r'):
            self.clicked_points = []
            self.mask_overlay = None

        elif key == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SamRealSenseTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()


if __name__ == '__main__':
    main()