import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import cv2
import numpy as np
import open3d as o3d

from ultralytics import YOLO


class YoloDepthPlane3DRealSense(Node):
    def __init__(self):
        super().__init__('yolo_depth_plane_3d_realsense')

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

        self.bridge = CvBridge()

        self.latest_rgb = None
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.yolo = YOLO(self.yolo_model_path)

        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/valve/plane_pose', 10)

        cv2.namedWindow('YOLO Depth Plane')
        cv2.namedWindow('Depth Selected Points')

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='3D Plane Viewer', width=900, height=700)

        self.o3d_initialized = False
        self.last_valid_points = None
        self.last_valid_plane = None

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

    def make_points_from_depth_band(self, depth_m, bbox, center_depth):
        x1, y1, x2, y2, conf = bbox

        roi = depth_m[y1:y2, x1:x2]

        valid = np.isfinite(roi)
        valid &= roi > 0.1
        valid &= roi < 10.0
        valid &= np.abs(roi - center_depth) <= self.depth_band_m

        ys, xs = np.where(valid)

        if len(xs) < 30:
            return None, None

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

        return points_3d, pixels

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

        if normal[2] > 0:
            normal = -normal

        d = -np.dot(normal, centroid)

        return normal, d, centroid, best_inliers, best_count

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

    def publish_pose(self, centroid, normal):
        qx, qy, qz, qw = self.normal_to_quaternion(normal)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'

        msg.pose.position.x = float(centroid[0])
        msg.pose.position.y = float(centroid[1])
        msg.pose.position.z = float(centroid[2])

        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.pose_pub.publish(msg)

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

    def make_plane_mesh(self, centroid, normal, size):
        normal = normal / np.linalg.norm(normal)

        ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        if abs(np.dot(normal, ref)) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        axis1 = np.cross(normal, ref)
        axis1 = axis1 / np.linalg.norm(axis1)

        axis2 = np.cross(normal, axis1)
        axis2 = axis2 / np.linalg.norm(axis2)

        s = size * 0.5

        v0 = centroid - axis1 * s - axis2 * s
        v1 = centroid + axis1 * s - axis2 * s
        v2 = centroid + axis1 * s + axis2 * s
        v3 = centroid - axis1 * s + axis2 * s

        vertices = np.array([v0, v1, v2, v3], dtype=np.float64)
        triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.paint_uniform_color([0.1, 0.4, 1.0])
        mesh.compute_vertex_normals()

        return mesh

    def make_normal_lineset(self, centroid, normal, length=0.15):
        p1 = centroid
        p2 = centroid + normal * length

        points = np.array([p1, p2], dtype=np.float64)
        lines = np.array([[0, 1]], dtype=np.int32)

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 0.0]], dtype=np.float64))

        return line_set

    def update_3d_viewer(self, points_3d, inliers, centroid, normal):
        all_cloud = o3d.geometry.PointCloud()
        all_cloud.points = o3d.utility.Vector3dVector(points_3d.astype(np.float64))
        all_cloud.paint_uniform_color([0.0, 0.8, 0.0])

        inlier_points = points_3d[inliers]
        inlier_cloud = o3d.geometry.PointCloud()
        inlier_cloud.points = o3d.utility.Vector3dVector(inlier_points.astype(np.float64))
        inlier_cloud.paint_uniform_color([1.0, 0.8, 0.0])

        plane_mesh = self.make_plane_mesh(centroid.astype(np.float64), normal.astype(np.float64), self.plane_size_m)
        normal_line = self.make_normal_lineset(centroid.astype(np.float64), normal.astype(np.float64))

        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0.0, 0.0, 0.0])

        self.vis.clear_geometries()
        self.vis.add_geometry(frame)
        self.vis.add_geometry(all_cloud)
        self.vis.add_geometry(inlier_cloud)
        self.vis.add_geometry(plane_mesh)
        self.vis.add_geometry(normal_line)

        opt = self.vis.get_render_option()
        opt.point_size = 3.0
        opt.mesh_show_back_face = True

        self.vis.poll_events()
        self.vis.update_renderer()

    def process(self):
        if self.latest_rgb is None:
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        if self.latest_depth is None:
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        if self.fx is None:
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        rgb = self.latest_rgb.copy()
        depth_m = self.depth_to_meter(self.latest_depth)

        if depth_m.shape[:2] != rgb.shape[:2]:
            cv2.putText(rgb, 'Depth is not aligned to color',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.waitKey(1)
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        bbox = self.detect_yolo(rgb)
        selected_view = np.zeros_like(rgb)

        if bbox is None:
            cv2.putText(rgb, 'YOLO: no detection',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.imshow('Depth Selected Points', selected_view)
            cv2.waitKey(1)
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        bbox = self.shrink_bbox(bbox, rgb.shape)

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
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        points_3d, pixels = self.make_points_from_depth_band(depth_m, bbox, center_depth)

        if points_3d is None:
            cv2.putText(rgb, f'Not enough depth points, z={center_depth:.3f}m',
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('YOLO Depth Plane', rgb)
            cv2.imshow('Depth Selected Points', selected_view)
            cv2.waitKey(1)
            self.vis.poll_events()
            self.vis.update_renderer()
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
            self.vis.poll_events()
            self.vis.update_renderer()
            return

        normal, d, centroid, inliers, inlier_count = plane

        self.publish_pose(centroid, normal)
        self.draw_normal_2d(rgb, centroid, normal)
        self.update_3d_viewer(points_3d, inliers, centroid, normal)

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
        self.vis.destroy_window()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthPlane3DRealSense()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()


if __name__ == '__main__':
    main()