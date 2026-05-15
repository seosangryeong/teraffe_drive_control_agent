import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import cv2
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from ultralytics import YOLO


class YoloPlaneCircle(Node):
    def __init__(self):
        super().__init__('YoloPlaneCircle')

        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('yolo_model', '/home/nuc/valve_yolo/runs/detect/train3/weights/best.pt')
        self.declare_parameter('conf_thres', 0.4)
        self.declare_parameter('depth_band_m', 0.05)
        self.declare_parameter('ransac_iter', 300)
        self.declare_parameter('ransac_thresh_m', 0.01)
        self.declare_parameter('max_points', 6000)
        self.declare_parameter('bbox_shrink', 0.90)
        self.declare_parameter('plane_size_m', 0.25)
        self.declare_parameter('circle_ratio', 0.6)
        self.declare_parameter('circle_points', 160)
        self.declare_parameter('graph_interval', 0.15)

        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.depth_band_m = float(self.get_parameter('depth_band_m').value)
        self.ransac_iter = int(self.get_parameter('ransac_iter').value)
        self.ransac_thresh_m = float(self.get_parameter('ransac_thresh_m').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.bbox_shrink = float(self.get_parameter('bbox_shrink').value)
        self.plane_size_m = float(self.get_parameter('plane_size_m').value)
        self.circle_ratio = float(self.get_parameter('circle_ratio').value)
        self.circle_points = int(self.get_parameter('circle_points').value)
        self.graph_interval = float(self.get_parameter('graph_interval').value)

        self.bridge = CvBridge()

        self.latest_rgb = None
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.graph_xlim = None
        self.graph_ylim = None
        self.graph_zlim = None
        self.last_graph_time = 0.0

        self.yolo = YOLO(self.yolo_model_path)

        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/valve/plane_pose', 10)
        self.start_pose_pub = self.create_publisher(PoseStamped, '/valve/circle_start_pose', 10)

        cv2.namedWindow('YOLO Depth Plane')
        cv2.namedWindow('Depth Selected Points')

        plt.ion()
        self.fig = plt.figure('3D Plane Graph')
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.subplots_adjust(right=0.76)
        self.fig.show()

        self.timer = self.create_timer(0.03, self.process)

        self.get_logger().info(f'rgb_topic: {self.rgb_topic}')
        self.get_logger().info(f'depth_topic: {self.depth_topic}')
        self.get_logger().info(f'camera_info_topic: {self.camera_info_topic}')
        self.get_logger().info(f'yolo_model: {self.yolo_model_path}')

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_depth = depth.copy()

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def detect_yolo(self, image):
        results = self.yolo.predict(source=image, conf=self.conf_thres, verbose=False)

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

    def shrink_bbox(self, bbox, image_shape):
        x1, y1, x2, y2, conf = bbox

        h, w = image_shape[:2]

        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        bw = (x2 - x1) * self.bbox_shrink
        bh = (y2 - y1) * self.bbox_shrink

        nx1 = int(cx - bw * 0.5)
        ny1 = int(cy - bh * 0.5)
        nx2 = int(cx + bw * 0.5)
        ny2 = int(cy + bh * 0.5)

        nx1 = int(np.clip(nx1, 0, w - 1))
        ny1 = int(np.clip(ny1, 0, h - 1))
        nx2 = int(np.clip(nx2, 0, w - 1))
        ny2 = int(np.clip(ny2, 0, h - 1))

        return [nx1, ny1, nx2, ny2, conf]

    def depth_to_meter(self, depth):
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * 0.001

        return depth.astype(np.float32)

    def get_center_depth(self, depth_m, cx_pix, cy_pix):
        h, w = depth_m.shape[:2]

        r = 5
        x1 = max(0, cx_pix - r)
        y1 = max(0, cy_pix - r)
        x2 = min(w, cx_pix + r + 1)
        y2 = min(h, cy_pix + r + 1)

        patch = depth_m[y1:y2, x1:x2]
        valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 10.0)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def pixel_to_3d(self, u, v, z):
        x = (float(u) - self.cx) * z / self.fx
        y = (float(v) - self.cy) * z / self.fy
        return np.array([x, y, z], dtype=np.float64)

    def make_points_from_depth_band(self, depth_m, bbox, center_depth):
        x1, y1, x2, y2, conf = bbox

        roi = depth_m[y1:y2, x1:x2]

        valid = np.isfinite(roi)
        valid &= roi > 0.1
        valid &= roi < 10.0
        valid &= np.abs(roi - center_depth) <= self.depth_band_m

        ys, xs = np.where(valid)

        if len(xs) < 30:
            return None, None, None

        xs_full = xs + x1
        ys_full = ys + y1
        zs = roi[ys, xs]

        if len(xs_full) > self.max_points:
            idx = np.random.choice(len(xs_full), self.max_points, replace=False)
            xs_full = xs_full[idx]
            ys_full = ys_full[idx]
            zs = zs[idx]

        X = (xs_full.astype(np.float32) - self.cx) * zs / self.fx
        Y = (ys_full.astype(np.float32) - self.cy) * zs / self.fy
        Z = zs.astype(np.float32)

        points_3d = np.column_stack((X, Y, Z))
        pixels = np.column_stack((xs_full, ys_full))

        depth_mask = np.zeros(depth_m.shape[:2], dtype=np.uint8)
        depth_mask[ys_full, xs_full] = 255

        return points_3d, pixels, depth_mask

    def fit_plane_from_3points(self, p1, p2, p3):
        v1 = p2 - p1
        v2 = p3 - p1

        n = np.cross(v1, v2)
        norm = np.linalg.norm(n)

        if norm < 1e-6:
            return None

        n = n / norm
        d = -np.dot(n, p1)

        return n, d

    def ransac_plane(self, points):
        if points is None or len(points) < 30:
            return None

        best_inliers = None
        best_count = 0

        n_points = len(points)

        for _ in range(self.ransac_iter):
            ids = np.random.choice(n_points, 3, replace=False)
            p1, p2, p3 = points[ids]

            plane = self.fit_plane_from_3points(p1, p2, p3)

            if plane is None:
                continue

            normal, d = plane

            distances = np.abs(points @ normal + d)
            inliers = distances < self.ransac_thresh_m
            count = int(np.sum(inliers))

            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_inliers is None or best_count < 30:
            return None

        inlier_points = points[best_inliers]
        centroid = np.mean(inlier_points, axis=0)

        centered = inlier_points - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)

        normal = vh[-1, :]
        normal = normal / np.linalg.norm(normal)

        if np.dot(normal, centroid) > 0.0:
            normal = -normal

        d = -np.dot(normal, centroid)

        return normal, d, centroid, best_inliers, best_count

    def project_center_to_plane(self, center_3d, normal, d):
        dist = np.dot(normal, center_3d) + d
        return center_3d - dist * normal

    def make_plane_axes(self, normal):
        normal = normal / np.linalg.norm(normal)

        ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        if abs(np.dot(normal, ref)) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        axis1 = np.cross(normal, ref)
        axis1 = axis1 / np.linalg.norm(axis1)

        axis2 = np.cross(normal, axis1)
        axis2 = axis2 / np.linalg.norm(axis2)

        return axis1, axis2

    def estimate_valve_radius_m(self, bbox, center_depth):
        x1, y1, x2, y2, conf = bbox
        pixel_radius = min(x2 - x1, y2 - y1) * 0.5
        radius_m = pixel_radius * center_depth / self.fx
        return float(radius_m)

    def make_circle_points_on_plane(self, center_3d, normal, radius_m, start_angle):
        axis1, axis2 = self.make_plane_axes(normal)

        points = []

        for i in range(self.circle_points):
            theta = start_angle + 2.0 * np.pi * float(i) / float(self.circle_points)
            p = center_3d + radius_m * np.cos(theta) * axis1 + radius_m * np.sin(theta) * axis2
            points.append(p)

        return np.array(points, dtype=np.float64)

    def find_start_point_on_circle_without_spoke(self, center_3d, normal, circle_radius_m, valve_radius_m, depth_mask):
        axis1, axis2 = self.make_plane_axes(normal)

        num_samples = 360
        occupancy = []
        points = []
        angles = []

        check_radius_m = circle_radius_m

        for i in range(num_samples):
            theta = 2.0 * np.pi * float(i) / float(num_samples)

            p = center_3d + check_radius_m * np.cos(theta) * axis1 + check_radius_m * np.sin(theta) * axis2
            uv = self.project_point(p)

            start_p = center_3d + circle_radius_m * np.cos(theta) * axis1 + circle_radius_m * np.sin(theta) * axis2

            points.append(start_p)
            angles.append(theta)

            if uv is None:
                occupancy.append(1)
                continue

            u, v = uv

            if u < 4 or v < 4 or u >= depth_mask.shape[1] - 4 or v >= depth_mask.shape[0] - 4:
                occupancy.append(1)
                continue

            patch = depth_mask[v - 4:v + 5, u - 4:u + 5]
            score = float(np.mean(patch > 0))

            if score > 0.05:
                occupancy.append(1)
            else:
                occupancy.append(0)

        occupancy = np.array(occupancy, dtype=np.uint8)

        empty_indices = np.where(occupancy == 0)[0]

        if len(empty_indices) == 0:
            best_idx = 0
            return points[best_idx], angles[best_idx], 1.0

        doubled = np.concatenate([occupancy, occupancy])

        best_start = None
        best_len = 0

        i = 0
        while i < len(doubled):
            if doubled[i] == 0:
                start = i
                while i < len(doubled) and doubled[i] == 0:
                    i += 1
                length = i - start

                if start < num_samples and length > best_len:
                    best_start = start
                    best_len = min(length, num_samples)
            else:
                i += 1

        if best_start is None:
            best_idx = int(empty_indices[0])
            return points[best_idx], angles[best_idx], 1.0

        mid_idx = (best_start + best_len // 2) % num_samples

        start_point = points[mid_idx]
        start_angle = angles[mid_idx]

        empty_ratio = float(best_len) / float(num_samples)

        return start_point, start_angle, empty_ratio

    def normal_to_quaternion(self, normal):
        z_axis = normal / np.linalg.norm(normal)

        ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        if abs(np.dot(z_axis, ref)) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        x_axis = np.cross(ref, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        r = np.column_stack((x_axis, y_axis, z_axis))

        trace = np.trace(r)

        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r[2, 1] - r[1, 2]) / s
            qy = (r[0, 2] - r[2, 0]) / s
            qz = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s

        q = np.array([qx, qy, qz, qw], dtype=np.float64)
        q = q / np.linalg.norm(q)

        return q

    def publish_pose(self, topic_pub, position, normal):
        qx, qy, qz, qw = self.normal_to_quaternion(normal)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        topic_pub.publish(msg)

    def project_point(self, p):
        x, y, z = p

        if z <= 0:
            return None

        u = int(self.fx * x / z + self.cx)
        v = int(self.fy * y / z + self.cy)

        return u, v

    def draw_normal_2d(self, image, centroid, normal, length=0.12):
        p1 = centroid
        p2 = centroid + normal * length

        uv1 = self.project_point(p1)
        uv2 = self.project_point(p2)

        if uv1 is None or uv2 is None:
            return

        cv2.arrowedLine(image, uv1, uv2, (0, 0, 255), 3, tipLength=0.25)

    def draw_circle_2d(self, image, circle_points, start_point):
        return

    def make_plane_surface_cm(self, centroid, normal, size_m):
        axis1, axis2 = self.make_plane_axes(normal)

        s = size_m * 0.5
        grid = np.linspace(-s, s, 12)

        xs = []
        ys = []
        zs = []

        for a in grid:
            row_x = []
            row_y = []
            row_z = []

            for b in grid:
                p = centroid + a * axis1 + b * axis2
                row_x.append(p[0] * 100.0)
                row_y.append(p[1] * 100.0)
                row_z.append(p[2] * 100.0)

            xs.append(row_x)
            ys.append(row_y)
            zs.append(row_z)

        return np.array(xs), np.array(ys), np.array(zs)

    def set_axes_equal_cm(self, ax, points_cm):
        if points_cm is None or len(points_cm) == 0:
            return

        x_min, y_min, z_min = np.min(points_cm, axis=0)
        x_max, y_max, z_max = np.max(points_cm, axis=0)

        x_mid = (x_min + x_max) * 0.5
        y_mid = (y_min + y_max) * 0.5
        z_mid = (z_min + z_max) * 0.5

        max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
        max_range = max(max_range, 20.0)

        r = max_range * 0.6

        ax.set_xlim(x_mid - r, x_mid + r)
        ax.set_ylim(y_mid - r, y_mid + r)
        ax.set_zlim(z_mid - r, z_mid + r)

    def update_3d_graph(self, points_3d, inliers, centroid, normal, circle_points, start_point, circle_center):
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_graph_time < self.graph_interval:
            return

        self.last_graph_time = now

        points_cm = points_3d.astype(np.float64) * 100.0
        inlier_cm = points_3d[inliers].astype(np.float64) * 100.0
        centroid_cm = centroid.astype(np.float64) * 100.0
        normal_cm_end = centroid_cm + normal.astype(np.float64) * 15.0

        self.ax.cla()

        if len(points_cm) > 0:
            self.ax.scatter(
                points_cm[:, 0],
                points_cm[:, 2],
                -points_cm[:, 1],
                s=2,
                c='green',
                alpha=0.25,
                label='depth candidates'
            )

        if len(inlier_cm) > 0:
            self.ax.scatter(
                inlier_cm[:, 0],
                inlier_cm[:, 2],
                -inlier_cm[:, 1],
                s=4,
                c='gold',
                alpha=0.8,
                label='plane inliers'
            )

        px, py, pz = self.make_plane_surface_cm(centroid, normal, self.plane_size_m)

        self.ax.plot_surface(
            px,
            pz,
            -py,
            color='dodgerblue',
            alpha=0.25,
            shade=False
        )

        self.ax.plot(
            [centroid_cm[0], normal_cm_end[0]],
            [centroid_cm[2], normal_cm_end[2]],
            [-centroid_cm[1], -normal_cm_end[1]],
            c='red',
            linewidth=3,
            label='normal'
        )

        self.ax.scatter(
            [0.0],
            [0.0],
            [0.0],
            s=100,
            c='blue',
            marker='s',
            label='camera'
        )

        all_for_limit = np.vstack([
            points_cm,
            centroid_cm.reshape(1, 3),
            normal_cm_end.reshape(1, 3)
        ])

        mapped_all = np.column_stack((
            all_for_limit[:, 0],
            all_for_limit[:, 2],
            -all_for_limit[:, 1]
        ))

        x_min, y_min, z_min = np.min(mapped_all, axis=0)
        x_max, y_max, z_max = np.max(mapped_all, axis=0)

        x_mid = (x_min + x_max) * 0.5
        y_mid = (y_min + y_max) * 0.5
        z_mid = (z_min + z_max) * 0.5

        x_range = max(x_max - x_min, 35.0)
        y_range = max(y_max - y_min, 35.0)
        z_range = max(z_max - z_min, 35.0)

        x_margin = x_range * 0.8
        y_margin = y_range * 0.8
        z_margin = z_range * 0.8

        self.graph_xlim = (x_mid - x_margin, x_mid + x_margin)
        self.graph_ylim = (y_mid - y_margin, y_mid + y_margin)
        self.graph_zlim = (z_mid - z_margin, z_mid + z_margin)

        self.ax.view_init(elev=15, azim=-90)

        try:
            self.ax.set_proj_type('persp')
        except Exception:
            pass

        try:
            self.ax.set_box_aspect((1.0, 1.5, 1.0))
        except Exception:
            pass

        self.ax.set_xlim(self.graph_xlim)
        self.ax.set_ylim(self.graph_ylim)
        self.ax.set_zlim(self.graph_zlim)

        self.ax.set_xlabel('X right [cm]')
        self.ax.set_ylabel('Z forward [cm]')
        self.ax.set_zlabel('Y up [cm]')

        self.ax.set_title('Camera Perspective View')
        self.ax.legend(
            loc='upper left',
            bbox_to_anchor=(1.03, 1.0),
            borderaxespad=0.0,
            fontsize=8,
            markerscale=0.8,
            labelspacing=0.4
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def process(self):
        if self.latest_rgb is None:
            plt.pause(0.001)
            return

        if self.latest_depth is None:
            plt.pause(0.001)
            return

        if self.fx is None:
            plt.pause(0.001)
            return

        rgb = self.latest_rgb.copy()
        depth_m = self.depth_to_meter(self.latest_depth)

        if depth_m.shape[:2] != rgb.shape[:2]:
            cv2.putText(rgb, 'Depth is not aligned to color',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.waitKey(1)
            plt.pause(0.001)
            return

        bbox_raw = self.detect_yolo(rgb)
        selected_view = np.zeros_like(rgb)

        if bbox_raw is None:
            cv2.putText(rgb, 'YOLO: no detection',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.imshow('Depth Selected Points', selected_view)
            cv2.waitKey(1)
            plt.pause(0.001)
            return

        bbox = self.shrink_bbox(bbox_raw, rgb.shape)

        x1, y1, x2, y2, conf = bbox

        valve_cx = int((x1 + x2) * 0.5)
        valve_cy = int((y1 + y2) * 0.5)

        center_depth = self.get_center_depth(depth_m, valve_cx, valve_cy)

        cv2.rectangle(rgb, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.circle(rgb, (valve_cx, valve_cy), 5, (0, 0, 255), -1)

        if center_depth is None:
            cv2.putText(rgb, 'Center depth invalid',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.imshow('Depth Selected Points', selected_view)
            cv2.waitKey(1)
            plt.pause(0.001)
            return

        center_3d = self.pixel_to_3d(valve_cx, valve_cy, center_depth)

        points_3d, pixels, depth_mask = self.make_points_from_depth_band(depth_m, bbox, center_depth)

        if points_3d is None:
            cv2.putText(rgb, f'Not enough depth points, z={center_depth:.3f}m',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.imshow('Depth Selected Points', selected_view)
            cv2.waitKey(1)
            plt.pause(0.001)
            return

        for u, v in pixels:
            cv2.circle(rgb, (int(u), int(v)), 1, (0, 255, 0), -1)
            cv2.circle(selected_view, (int(u), int(v)), 1, (0, 255, 0), -1)

        plane = self.ransac_plane(points_3d)

        if plane is None:
            cv2.putText(rgb, f'Plane RANSAC failed, pts={len(points_3d)}',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.imshow('Depth Selected Points', selected_view)
            cv2.waitKey(1)
            plt.pause(0.001)
            return

        normal, d, centroid, inliers, inlier_count = plane

        circle_center = self.project_center_to_plane(center_3d, normal, d)

        valve_radius_m = self.estimate_valve_radius_m(bbox_raw, center_depth)
        circle_radius_m = valve_radius_m * self.circle_ratio

        start_point, start_angle, start_score = self.find_start_point_on_circle_without_spoke(
            circle_center,
            normal,
            circle_radius_m,
            valve_radius_m,
            depth_mask
        )

        circle_points = self.make_circle_points_on_plane(
            circle_center,
            normal,
            circle_radius_m,
            start_angle
        )

        self.publish_pose(self.pose_pub, centroid, normal)
        self.publish_pose(self.start_pose_pub, start_point, normal)

        self.draw_normal_2d(rgb, centroid, normal)
        self.draw_circle_2d(rgb, circle_points, start_point)
        self.update_3d_graph(points_3d, inliers, centroid, normal, circle_points, start_point, circle_center)

        inlier_pixels = pixels[inliers]

        for u, v in inlier_pixels:
            cv2.circle(rgb, (int(u), int(v)), 1, (0, 255, 255), -1)
            cv2.circle(selected_view, (int(u), int(v)), 1, (0, 255, 255), -1)

        nx, ny, nz = normal

        text1 = f'YOLO {conf:.2f}, center_z={center_depth:.3f}m, pts={len(points_3d)}, inliers={inlier_count}'
        text2 = f'normal=({nx:.3f}, {ny:.3f}, {nz:.3f})'

        cv2.putText(rgb, text1,
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        cv2.putText(rgb, text2,
                    (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        cv2.imshow('YOLO Depth Plane', rgb)
        cv2.imshow('Depth Selected Points', selected_view)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        plt.close(self.fig)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloPlaneCircle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()


if __name__ == '__main__':
    main()