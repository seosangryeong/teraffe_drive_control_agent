#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import struct
import random


class RealsenseYoloValveCircleViewer(Node):
    def __init__(self):
        super().__init__('realsense_yolo_valve_circle_viewer')

        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.camera_info_topic = '/camera/camera/color/camera_info'

        self.model_path = '/home/nuc/valve_yolo/runs/detect/train3/weights/best.pt'
        self.target_class_id = 4

        self.conf_thres = 0.25
        self.imgsz = 640
        self.infer_interval = 2

        self.depth_min = 0.1
        self.depth_max = 2.5
        self.depth_margin = 0.06

        self.ransac_iter = 120
        self.ransac_dist_thres = 5.0
        self.min_circle_inliers = 60

        self.max_edge_points = 700
        self.publish_pc_interval = 3

        self.spoke_count = 5
        self.spoke_angle_step = 3
        self.spoke_r_inner_ratio = 0.30
        self.spoke_r_outer_ratio = 0.72
        self.spoke_band_ratio = 0.04
        self.spoke_detect_interval = 4
        self.last_spokes = []

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        self.latest_rgb = None
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_ready = False

        self.frame_count = 0
        self.last_vis = None
        self.last_box = None

        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 1)

        self.roi_pc_pub = self.create_publisher(PointCloud2, '/valve_roi_points', 1)

        self.timer = self.create_timer(0.03, self.process)

        cv2.namedWindow('rgb_view', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth_view', cv2.WINDOW_NORMAL)

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

    def select_best_box(self, result, img_shape):
        h, w = img_shape[:2]
        img_cx = w * 0.5
        img_cy = h * 0.5

        best = None
        best_score = -1e18

        if result.boxes is None or len(result.boxes) == 0:
            return None

        for box in result.boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())

            if cls_id != self.target_class_id or conf < self.conf_thres:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            x1 = max(0, int(x1))
            y1 = max(0, int(y1))
            x2 = min(w - 1, int(x2))
            y2 = min(h - 1, int(y2))

            bw = x2 - x1
            bh = y2 - y1

            if bw < 20 or bh < 20:
                continue

            area = bw * bh
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            dist = np.hypot(cx - img_cx, cy - img_cy)

            score = conf * 1000.0 + area * 0.01 - dist * 0.3

            if score > best_score:
                best_score = score
                best = (x1, y1, x2, y2, conf)

        return best

    # "YOLO 박스 내부에서 밸브에 해당하는 depth 픽셀이 가장 많이 존재한다"고 가정하고 밸브 외부 데이터를 다 제거
    def make_valve_mask(self, depth_roi):
        valid_mask = (
            (depth_roi > self.depth_min) &
            (depth_roi < self.depth_max) &
            np.isfinite(depth_roi)
        )

        valid_depths = depth_roi[valid_mask]

        if len(valid_depths) == 0:
            return np.zeros_like(depth_roi, dtype=bool), None

        hist, bins = np.histogram(valid_depths, bins=60)
        peak_idx = np.argmax(hist)
        target_depth = 0.5 * (bins[peak_idx] + bins[peak_idx + 1])

        valve_mask = (
            valid_mask &
            (depth_roi >= target_depth - self.depth_margin) &
            (depth_roi <= target_depth + self.depth_margin)
        )

        kernel = np.ones((5, 5), np.uint8)
        valve_mask_u8 = valve_mask.astype(np.uint8) * 255
        valve_mask_u8 = cv2.morphologyEx(valve_mask_u8, cv2.MORPH_OPEN, kernel)
        valve_mask_u8 = cv2.morphologyEx(valve_mask_u8, cv2.MORPH_CLOSE, kernel, iterations=2)

        return valve_mask_u8 > 0, target_depth

    # Ransac을 하기 위한 3개 포인트 추출
    def circle_from_3pts(self, p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        temp = x2 * x2 + y2 * y2
        bc = (x1 * x1 + y1 * y1 - temp) / 2.0
        cd = (temp - x3 * x3 - y3 * y3) / 2.0
        det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)

        if abs(det) < 1e-6:
            return None

        cx = (bc * (y2 - y3) - cd * (y1 - y2)) / det
        cy = ((x1 - x2) * cd - (x2 - x3) * bc) / det
        r = np.hypot(cx - x1, cy - y1)

        if not np.isfinite(cx) or not np.isfinite(cy) or not np.isfinite(r):
            return None

        return cx, cy, r

    # ransac으로 골라낸 원을 least squares로 다시 피팅
    def fit_circle_least_squares(self, pts):
        x = pts[:, 0].astype(np.float64)
        y = pts[:, 1].astype(np.float64)

        A = np.column_stack((2 * x, 2 * y, np.ones_like(x)))
        b = x * x + y * y

        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except Exception:
            return None

        cx, cy, c = sol
        r = np.sqrt(cx * cx + cy * cy + c)

        if not np.isfinite(cx) or not np.isfinite(cy) or not np.isfinite(r):
            return None

        return float(cx), float(cy), float(r)

    def is_circle_inside_box(self, cx, cy, r, roi_w, roi_h):
        if cx < 0 or cx >= roi_w or cy < 0 or cy >= roi_h:
            return False

        if r < min(roi_w, roi_h) * 0.12:
            return False

        if r > min(roi_w, roi_h) * 0.62:
            return False

        margin = 4.0

        if cx - r < -margin:
            return False

        if cy - r < -margin:
            return False

        if cx + r > roi_w + margin:
            return False

        if cy + r > roi_h + margin:
            return False

        return True

    # ransac 알고리즘
    def ransac_circle(self, edge_pts, roi_w, roi_h):
        if len(edge_pts) < self.min_circle_inliers:
            return None

        best_circle = None
        best_inliers = None
        best_count = 0

        pts = edge_pts.astype(np.float32)

        for _ in range(self.ransac_iter):
            ids = np.random.choice(len(pts), 3, replace=False)
            circle = self.circle_from_3pts(pts[ids[0]], pts[ids[1]], pts[ids[2]])

            if circle is None:
                continue

            cx, cy, r = circle

            if not self.is_circle_inside_box(cx, cy, r, roi_w, roi_h):
                continue

            d = np.abs(np.sqrt((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2) - r)
            inliers = d < self.ransac_dist_thres
            count = int(np.sum(inliers))

            if count > best_count:
                best_count = count
                best_circle = circle
                best_inliers = inliers

        if best_circle is None or best_count < self.min_circle_inliers:
            return None

        refined = self.fit_circle_least_squares(pts[best_inliers])

        if refined is not None:
            cx, cy, r = refined

            if self.is_circle_inside_box(cx, cy, r, roi_w, roi_h):
                return refined, best_count

        return best_circle, best_count

    # 밸브 외곽선 점들 추출 (ransac을 하기 위해)
    def extract_edge_points(self, valve_mask):
        mask_u8 = valve_mask.astype(np.uint8) * 255

        contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) == 0:
            return None

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        pts_all = []

        for c in contours[:3]:
            if cv2.contourArea(c) < 50:
                continue
            pts = c.reshape(-1, 2)
            pts_all.append(pts)

        if len(pts_all) == 0:
            return None

        edge_pts = np.vstack(pts_all)

        if len(edge_pts) > self.max_edge_points:
            idx = np.random.choice(len(edge_pts), self.max_edge_points, replace=False)
            edge_pts = edge_pts[idx]

        return edge_pts

    def detect_spoke_centerlines(self, rgb_roi, depth_roi, valve_mask, cx_roi, cy_roi, radius):
        roi_h, roi_w = depth_roi.shape[:2]

        if radius <= 10:
            return []

        gray = cv2.cvtColor(rgb_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 40, 120)

        mask_u8 = valve_mask.astype(np.uint8) * 255
        edges = cv2.bitwise_and(edges, mask_u8)

        r_inner = max(5.0, radius * self.spoke_r_inner_ratio)
        r_outer = min(radius * self.spoke_r_outer_ratio, radius - 8.0)

        if r_outer <= r_inner:
            return []

        rs = np.linspace(r_inner, r_outer, 28)

        band = int(max(3, radius * self.spoke_band_ratio))
        offsets = np.arange(-band, band + 1)

        best_angle = None
        best_score = -1.0

        for angle_deg in range(0, 360, self.spoke_angle_step):
            theta = np.deg2rad(angle_deg)

            dx = np.cos(theta)
            dy = np.sin(theta)
            px = -dy
            py = dx

            xs = []
            ys = []

            for rv in rs:
                bx = cx_roi + dx * rv
                by = cy_roi + dy * rv

                for off in offsets:
                    x = int(round(bx + px * off))
                    y = int(round(by + py * off))

                    if 0 <= x < roi_w and 0 <= y < roi_h:
                        xs.append(x)
                        ys.append(y)

            if len(xs) < 20:
                continue

            xs = np.array(xs, dtype=np.int32)
            ys = np.array(ys, dtype=np.int32)

            mask_count = np.count_nonzero(mask_u8[ys, xs])
            edge_count = np.count_nonzero(edges[ys, xs])

            mask_ratio = mask_count / max(1, len(xs))
            edge_ratio = edge_count / max(1, len(xs))

            if mask_ratio < 0.20:
                continue

            score = mask_ratio * 100.0 + edge_ratio * 60.0

            if score > best_score:
                best_score = score
                best_angle = angle_deg

        if best_angle is None:
            return []

        selected = []

        for k in range(self.spoke_count):
            angle_deg = (best_angle + k * (360.0 / self.spoke_count)) % 360.0
            theta = np.deg2rad(angle_deg)

            x_start = int(round(cx_roi + np.cos(theta) * r_inner))
            y_start = int(round(cy_roi + np.sin(theta) * r_inner))
            x_end = int(round(cx_roi + np.cos(theta) * r_outer))
            y_end = int(round(cy_roi + np.sin(theta) * r_outer))

            selected.append((angle_deg, best_score, (x_start, y_start), (x_end, y_end)))

        selected = sorted(selected, key=lambda x: x[0])

        return selected

    def create_point_cloud_and_mask(self, rgb, depth, box):
        x1, y1, x2, y2, _ = box

        depth_roi = depth[y1:y2, x1:x2]
        rgb_roi = rgb[y1:y2, x1:x2]

        valve_mask, target_depth = self.make_valve_mask(depth_roi)

        u, v = np.meshgrid(np.arange(x1, x2), np.arange(y1, y2))

        u_valid = u[valve_mask]
        v_valid = v[valve_mask]
        z_valid = depth_roi[valve_mask]

        r_valid = rgb_roi[valve_mask, 2]
        g_valid = rgb_roi[valve_mask, 1]
        b_valid = rgb_roi[valve_mask, 0]

        x = (u_valid - self.cx) * z_valid / self.fx
        y = (v_valid - self.cy) * z_valid / self.fy
        z = z_valid

        points = []

        for i in range(len(x)):
            a = 255
            rgb_packed = struct.unpack(
                'I',
                struct.pack('BBBB', b_valid[i], g_valid[i], r_valid[i], a)
            )[0]
            points.append([x[i], y[i], z[i], rgb_packed])

        return points, valve_mask, target_depth

    def publish_point_cloud(self, points):
        if len(points) == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_color_optical_frame'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        point_data = bytearray()

        for p in points:
            point_data.extend(struct.pack('fffI', p[0], p[1], p[2], p[3]))

        pc_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * len(points),
            data=point_data
        )

        self.roi_pc_pub.publish(pc_msg)

    def process(self):
        if self.latest_rgb is None or self.latest_depth is None or not self.camera_info_ready:
            return

        rgb = self.latest_rgb.copy()
        depth = self.latest_depth.copy()

        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warn(
                f'rgb/depth size mismatch: rgb={rgb.shape[:2]}, depth={depth.shape[:2]}'
            )
            return

        h, w = depth.shape

        self.frame_count += 1

        result_rgb = rgb.copy()

        depth_vis = depth.copy()
        depth_vis = np.clip(depth_vis, 0.3, 2.5)
        depth_norm = ((depth_vis - 0.3) / (2.5 - 0.3) * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_TURBO)

        isolated_depth_color = np.zeros_like(depth_color)

        best_box = None

        if self.frame_count % self.infer_interval == 0 or self.last_box is None:
            yolo_result = self.model(rgb, imgsz=self.imgsz, conf=self.conf_thres, verbose=False)[0]
            best_box = self.select_best_box(yolo_result, rgb.shape)
            self.last_box = best_box
        else:
            best_box = self.last_box

        if best_box is not None:
            x1, y1, x2, y2, conf = best_box
            roi_w = x2 - x1
            roi_h = y2 - y1

            cv2.rectangle(result_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                result_rgb,
                f'valve {conf:.2f}',
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            points, valve_mask, target_depth = self.create_point_cloud_and_mask(rgb, depth, best_box)

            depth_roi = depth[y1:y2, x1:x2]
            rgb_roi = rgb[y1:y2, x1:x2]

            isolated_depth_color[y1:y2, x1:x2][valve_mask] = depth_color[y1:y2, x1:x2][valve_mask]

            edge_pts = self.extract_edge_points(valve_mask)

            spokes = []

            if edge_pts is not None:
                ransac_result = self.ransac_circle(edge_pts, roi_w, roi_h)

                for p in edge_pts:
                    px = int(p[0] + x1)
                    py = int(p[1] + y1)
                    cv2.circle(isolated_depth_color, (px, py), 1, (255, 255, 255), -1)

                if ransac_result is not None:
                    circle, inlier_count = ransac_result
                    cx_roi, cy_roi, radius = circle

                    cx_img = int(round(cx_roi + x1))
                    cy_img = int(round(cy_roi + y1))
                    radius_i = int(round(radius))

                    cv2.circle(result_rgb, (cx_img, cy_img), radius_i, (0, 0, 255), 3)
                    cv2.circle(result_rgb, (cx_img, cy_img), 4, (0, 255, 255), -1)

                    cv2.circle(isolated_depth_color, (cx_img, cy_img), radius_i, (0, 0, 255), 3)
                    cv2.circle(isolated_depth_color, (cx_img, cy_img), 4, (0, 255, 255), -1)

                    if self.frame_count % self.spoke_detect_interval == 0 or len(self.last_spokes) == 0:
                        spokes = self.detect_spoke_centerlines(
                            rgb_roi,
                            depth_roi,
                            valve_mask,
                            cx_roi,
                            cy_roi,
                            radius
                        )
                        self.last_spokes = spokes
                    else:
                        spokes = self.last_spokes

                    for _, _, p_start, p_end in spokes:
                        sx, sy = p_start
                        ex, ey = p_end

                        sx_img = int(sx + x1)
                        sy_img = int(sy + y1)
                        ex_img = int(ex + x1)
                        ey_img = int(ey + y1)

                        cv2.line(result_rgb, (sx_img, sy_img), (ex_img, ey_img), (255, 0, 255), 4)
                        cv2.line(isolated_depth_color, (sx_img, sy_img), (ex_img, ey_img), (255, 0, 255), 4)

                    cv2.putText(
                        result_rgb,
                        f'circle inliers={inlier_count}, centerlines={len(spokes)}',
                        (x1, min(h - 10, y2 + 25)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2
                    )

            if self.frame_count % self.publish_pc_interval == 0:
                self.publish_point_cloud(points)

        self.last_vis = {'rgb': result_rgb, 'depth': isolated_depth_color}
        self.show(result_rgb, isolated_depth_color)

    def show(self, rgb, depth_color):
        rgb_disp = cv2.resize(rgb, (960, 540))
        depth_disp = cv2.resize(depth_color, (960, 540))

        cv2.imshow('rgb_view', rgb_disp)
        cv2.imshow('depth_view', depth_disp)


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseYoloValveCircleViewer()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()