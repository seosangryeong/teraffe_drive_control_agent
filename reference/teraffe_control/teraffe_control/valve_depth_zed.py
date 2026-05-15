import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class ZedValveCircleViewer(Node):
    def __init__(self):
        super().__init__('zed_valve_circle_viewer')

        self.rgb_topic = '/zed/zed_node/rgb/color/rect/image'
        self.depth_topic = '/zed/zed_node/depth/depth_registered'

        self.bridge = CvBridge()

        self.depth_min = 0.3
        self.depth_max = 0.5
        self.min_contour_area = 200

        self.latest_rgb = None
        self.latest_depth = None

        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.timer = self.create_timer(0.03, self.process)

        cv2.namedWindow('rgb_view', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth_view', cv2.WINDOW_NORMAL)
        cv2.namedWindow('circle_view', cv2.WINDOW_NORMAL)

        self.get_logger().info(f'rgb topic: {self.rgb_topic}')
        self.get_logger().info(f'depth topic: {self.depth_topic}')

    def rgb_callback(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rgb = np.array(rgb)
        except Exception as e:
            self.get_logger().error(f'rgb callback failed: {e}')

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = np.array(depth)

            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            else:
                depth = depth.astype(np.float32)

            depth[~np.isfinite(depth)] = 0.0
            depth[depth < 0.0] = 0.0

            self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f'depth callback failed: {e}')

    def make_depth_visual(self, depth):
        h, w = depth.shape
        valid = np.isfinite(depth) & (depth > 0.0)

        if not np.any(valid):
            return np.zeros((h, w, 3), dtype=np.uint8)

        depth_vis = depth.copy()
        depth_vis[~valid] = self.depth_max
        depth_vis = np.clip(depth_vis, self.depth_min, self.depth_max)

        depth_norm = ((depth_vis - self.depth_min) / (self.depth_max - self.depth_min) * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_TURBO)
        depth_color[~valid] = (0, 0, 0)

        return depth_color

    def make_mask(self, depth):
        valid = np.isfinite(depth) & (depth > 0.0)
        mask = valid & (depth >= self.depth_min) & (depth <= self.depth_max)
        mask = (mask.astype(np.uint8) * 255)

        kernel_open = np.ones((3, 3), np.uint8)
        kernel_close = np.ones((7, 7), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        return mask

    def choose_best_contour(self, contours, img_shape):
        h, w = img_shape[:2]
        img_cx = w / 2.0
        img_cy = h / 2.0

        best_cnt = None
        best_score = -1e18

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 1e-6:
                continue

            circularity = 4.0 * np.pi * area / (perimeter * perimeter)

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if radius < 10:
                continue

            circle_area = np.pi * radius * radius
            fill_ratio = area / circle_area if circle_area > 1e-6 else 0.0

            center_dist = np.hypot(cx - img_cx, cy - img_cy)
            center_score = 1.0 - center_dist / np.hypot(w, h)
            center_score = max(center_score, 0.0)

            score = 0.0
            score += area * 0.01
            score += circularity * 200.0
            score += fill_ratio * 200.0
            score += center_score * 50.0

            if score > best_score:
                best_score = score
                best_cnt = cnt

        return best_cnt

    def process(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        depth = self.latest_depth.copy()
        rgb = self.latest_rgb.copy()

        h, w = depth.shape

        if rgb.shape[0] != h or rgb.shape[1] != w:
            rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_LINEAR)

        depth_color = self.make_depth_visual(depth)
        mask = self.make_mask(depth)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        result_rgb = rgb.copy()

        best_cnt = self.choose_best_contour(contours, depth.shape)

        if best_cnt is not None:
            cv2.drawContours(result_rgb, [best_cnt], -1, (255, 255, 255), 2)

            (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
            cxi = int(round(cx))
            cyi = int(round(cy))
            ri = int(round(radius))

            cxi = int(np.clip(cxi, 0, w - 1))
            cyi = int(np.clip(cyi, 0, h - 1))

            cv2.circle(result_rgb, (cxi, cyi), ri, (0, 0, 255), 3)
            cv2.circle(result_rgb, (cxi, cyi), 4, (0, 255, 0), -1)

            center_depth = depth[cyi, cxi]

            cv2.putText(
                result_rgb,
                f'center=({cxi},{cyi}) radius={ri}px',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            if np.isfinite(center_depth) and center_depth > 0:
                cv2.putText(
                    result_rgb,
                    f'depth={center_depth:.3f} m',
                    (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2
                )
        else:
            cv2.putText(
                result_rgb,
                'no valid circle',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2
            )

        self.show(rgb, depth_color, result_rgb)

    def show(self, rgb, depth_color, result_rgb):
        rgb_disp = cv2.resize(rgb, (960, 540))
        depth_disp = cv2.resize(depth_color, (960, 540))
        result_disp = cv2.resize(result_rgb, (960, 540))

        cv2.imshow('rgb_view', rgb_disp)
        cv2.imshow('depth_view', depth_disp)
        cv2.imshow('circle_view', result_disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ZedValveCircleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()