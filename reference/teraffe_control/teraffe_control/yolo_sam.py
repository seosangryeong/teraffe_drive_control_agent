import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch

from ultralytics import YOLO
from segment_anything import sam_model_registry, SamPredictor


class YoloSamRealSense(Node):
    def __init__(self):
        super().__init__('yolo_sam_realsense')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        # self.declare_parameter('image_topic', '/rgb')
        self.declare_parameter('yolo_model', '/home/nuc/valve_yolo/runs/detect/train3/weights/best.pt')
        self.declare_parameter('sam_checkpoint', '/home/nuc/sam/sam_vit_h_4b8939.pth')
        self.declare_parameter('model_type', 'vit_h')
        self.declare_parameter('conf_thres', 0.4)
        self.declare_parameter('ring_radius_ratio', 0.44)

        self.image_topic = self.get_parameter('image_topic').value
        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.sam_checkpoint = self.get_parameter('sam_checkpoint').value
        self.model_type = self.get_parameter('model_type').value
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.ring_radius_ratio = float(self.get_parameter('ring_radius_ratio').value)

        self.bridge = CvBridge()
        self.latest_image = None
        self.display_image = None
        self.auto_points = []
        self.mask_overlay = None
        self.latest_bbox = None

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        self.yolo = YOLO(self.yolo_model_path)

        sam = sam_model_registry[self.model_type](checkpoint=self.sam_checkpoint)
        sam.to(device=self.device)
        self.predictor = SamPredictor(sam)

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        cv2.namedWindow('YOLO SAM RealSense')
        self.timer = self.create_timer(0.03, self.show_image)

        self.get_logger().info(f'image_topic: {self.image_topic}')
        self.get_logger().info(f'yolo_model: {self.yolo_model_path}')
        self.get_logger().info(f'sam_checkpoint: {self.sam_checkpoint}')
        self.get_logger().info(f'model_type: {self.model_type}')
        self.get_logger().info(f'device: {self.device}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = frame.copy()

    def detect_yolo(self, image):
        results = self.yolo.predict(
            source=image,
            conf=self.conf_thres,
            verbose=False
        )

        if len(results) == 0:
            return None

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return None

        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()

        best_idx = int(np.argmax(confs))
        x1, y1, x2, y2 = boxes[best_idx]

        h, w = image.shape[:2]

        x1 = int(np.clip(x1, 0, w - 1))
        y1 = int(np.clip(y1, 0, h - 1))
        x2 = int(np.clip(x2, 0, w - 1))
        y2 = int(np.clip(y2, 0, h - 1))

        return [x1, y1, x2, y2, float(confs[best_idx])]

    def make_ring_points(self, bbox):
        x1, y1, x2, y2, conf = bbox

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        bw = x2 - x1
        bh = y2 - y1

        r = int(min(bw, bh) * self.ring_radius_ratio)

        angles_deg = [20, 150, 280]

        points = []

        for deg in angles_deg:
            rad = np.deg2rad(deg)

            x = int(cx + r * np.cos(rad))
            y = int(cy + r * np.sin(rad))

            x = int(np.clip(x, x1 + 3, x2 - 3))
            y = int(np.clip(y, y1 + 3, y2 - 3))

            points.append((x, y))

        return points

    def update_yolo_points(self):
        if self.latest_image is None:
            return False

        image_bgr = self.latest_image.copy()
        bbox = self.detect_yolo(image_bgr)

        if bbox is None:
            self.latest_bbox = None
            self.auto_points = []
            return False

        self.latest_bbox = bbox
        self.auto_points = self.make_ring_points(bbox)

        return True

    def run_sam_once(self):
        if self.latest_image is None:
            return

        ok = self.update_yolo_points()

        if not ok or len(self.auto_points) == 0:
            self.get_logger().warn('YOLO bbox 또는 자동 점을 찾지 못했습니다.')
            return

        image_bgr = self.latest_image.copy()
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        self.predictor.set_image(image_rgb)

        input_point = np.array(self.auto_points)
        input_label = np.ones(len(self.auto_points), dtype=np.int32)

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

        if self.latest_bbox is not None:
            x1, y1, x2, y2, conf = self.latest_bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 0, 0), 2)

        for x, y in self.auto_points:
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

            self.update_yolo_points()

            if self.latest_bbox is not None:
                x1, y1, x2, y2, conf = self.latest_bbox
                cv2.rectangle(show, (x1, y1), (x2, y2), (255, 0, 0), 2)

                cv2.putText(show, f'YOLO conf: {conf:.2f}',
                            (x1, max(30, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            for x, y in self.auto_points:
                cv2.circle(show, (x, y), 5, (0, 0, 255), -1)

        cv2.putText(show, 's: run SAM, r: reset, q: quit',
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('YOLO SAM RealSense', show)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self.run_sam_once()

        elif key == ord('r'):
            self.auto_points = []
            self.latest_bbox = None
            self.mask_overlay = None

        elif key == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = YoloSamRealSense()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()


if __name__ == '__main__':
    main()