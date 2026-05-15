import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2


class RealsenseValvePoseViewer(Node):
    def __init__(self):
        super().__init__('realsense_valve_pose_viewer')

        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.camera_info_topic = '/camera/camera/color/camera_info'

        self.bridge = CvBridge()

        self.depth_min = 0.25
        self.depth_max = 1.20
        self.min_contour_area = 300
        self.min_radius_px = 15
        self.plane_max_distance = 0.015

        self.latest_rgb = None
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_ready = False

        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.timer = self.create_timer(0.05, self.process)

        cv2.namedWindow('rgb_view', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth_view', cv2.WINDOW_NORMAL)
        cv2.namedWindow('result_view', cv2.WINDOW_NORMAL)

        self.get_logger().info(f'rgb topic: {self.rgb_topic}')
        self.get_logger().info(f'depth topic: {self.depth_topic}')
        self.get_logger().info(f'camera_info topic: {self.camera_info_topic}')

    def camera_info_callback(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_info_ready = True

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
            if radius < self.min_radius_px:
                continue

            circle_area = np.pi * radius * radius
            fill_ratio = area / circle_area if circle_area > 1e-6 else 0.0

            center_dist = np.hypot(cx - img_cx, cy - img_cy)
            center_score = 1.0 - center_dist / np.hypot(w, h)
            center_score = max(center_score, 0.0)

            score = 0.0
            score += area * 0.01
            score += circularity * 250.0
            score += fill_ratio * 180.0
            score += center_score * 40.0

            if score > best_score:
                best_score = score
                best_cnt = cnt

        return best_cnt

    def depth_to_points(self, us, vs, zs):
        xs = (us - self.cx) * zs / self.fx
        ys = (vs - self.cy) * zs / self.fy
        pts = np.stack([xs, ys, zs], axis=1)
        return pts

    def estimate_plane_and_center(self, depth, contour, draw_img):
        h, w = depth.shape

        filled = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(filled, [contour], -1, 255, thickness=-1)

        ys, xs = np.where(filled > 0)
        if len(xs) < 50:
            return None

        zs = depth[ys, xs]
        valid = np.isfinite(zs) & (zs > self.depth_min) & (zs < self.depth_max)
        xs = xs[valid]
        ys = ys[valid]
        zs = zs[valid]

        if len(xs) < 50:
            return None

        pts = self.depth_to_points(xs.astype(np.float32), ys.astype(np.float32), zs.astype(np.float32))

        mean = np.mean(pts, axis=0)
        centered = pts - mean

        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return None

        normal = vh[-1]
        normal = normal / (np.linalg.norm(normal) + 1e-8)

        if normal[2] > 0:
            normal = -normal

        distances = np.abs((pts - mean) @ normal)
        inlier_mask = distances < self.plane_max_distance
        pts_inlier = pts[inlier_mask]

        if len(pts_inlier) < 30:
            pts_inlier = pts

        center_3d = np.mean(pts_inlier, axis=0)

        plane_centered = pts_inlier - np.mean(pts_inlier, axis=0)
        try:
            _, _, vh2 = np.linalg.svd(plane_centered, full_matrices=False)
            normal = vh2[-1]
            normal = normal / (np.linalg.norm(normal) + 1e-8)
            if normal[2] > 0:
                normal = -normal
        except np.linalg.LinAlgError:
            pass

        c_u = int(round(center_3d[0] * self.fx / center_3d[2] + self.cx))
        c_v = int(round(center_3d[1] * self.fy / center_3d[2] + self.cy))

        c_u = int(np.clip(c_u, 0, w - 1))
        c_v = int(np.clip(c_v, 0, h - 1))

        axis_len = 0.10
        tip_3d = center_3d + normal * axis_len

        if tip_3d[2] > 1e-6:
            tip_u = int(round(tip_3d[0] * self.fx / tip_3d[2] + self.cx))
            tip_v = int(round(tip_3d[1] * self.fy / tip_3d[2] + self.cy))
            tip_u = int(np.clip(tip_u, 0, w - 1))
            tip_v = int(np.clip(tip_v, 0, h - 1))
            cv2.arrowedLine(draw_img, (c_u, c_v), (tip_u, tip_v), (255, 0, 0), 3, tipLength=0.2)

        cv2.circle(draw_img, (c_u, c_v), 5, (0, 255, 0), -1)

        return {
            'center_3d': center_3d,
            'normal': normal,
            'center_px': (c_u, c_v),
            'num_points': len(pts_inlier)
        }

    def process(self):
        if self.latest_rgb is None or self.latest_depth is None or not self.camera_info_ready:
            return

        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()

        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warn('rgb/depth shape mismatch')
            return

        depth_color = self.make_depth_visual(depth)
        mask = self.make_mask(depth)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        result = rgb.copy()
        best_cnt = self.choose_best_contour(contours, depth.shape)

        if best_cnt is None:
            cv2.putText(
                result,
                'no valid contour',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2
            )
            self.show(rgb, depth_color, result)
            return

        cv2.drawContours(result, [best_cnt], -1, (255, 255, 255), 2)

        (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
        cxi = int(round(cx))
        cyi = int(round(cy))
        ri = int(round(radius))

        cv2.circle(result, (cxi, cyi), ri, (0, 0, 255), 2)
        cv2.circle(result, (cxi, cyi), 4, (0, 255, 255), -1)

        pose = self.estimate_plane_and_center(depth, best_cnt, result)

        if pose is None:
            cv2.putText(
                result,
                'pose estimation failed',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2
            )
            self.show(rgb, depth_color, result)
            return

        center_3d = pose['center_3d']
        normal = pose['normal']
        center_px = pose['center_px']
        num_points = pose['num_points']

        cv2.putText(
            result,
            f'center_px=({center_px[0]},{center_px[1]})',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2
        )

        cv2.putText(
            result,
            f'center_3d=({center_3d[0]:.3f}, {center_3d[1]:.3f}, {center_3d[2]:.3f}) m',
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            result,
            f'normal=({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})',
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            result,
            f'inlier_points={num_points}',
            (20, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        self.get_logger().info(
            f'center_3d=({center_3d[0]:.4f}, {center_3d[1]:.4f}, {center_3d[2]:.4f}), '
            f'normal=({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})'
        )

        self.show(rgb, depth_color, result)

    def show(self, rgb, depth_color, result):
        rgb_disp = cv2.resize(rgb, (960, 540))
        depth_disp = cv2.resize(depth_color, (960, 540))
        result_disp = cv2.resize(result, (960, 540))

        cv2.imshow('rgb_view', rgb_disp)
        cv2.imshow('depth_view', depth_disp)
        cv2.imshow('result_view', result_disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseValvePoseViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()