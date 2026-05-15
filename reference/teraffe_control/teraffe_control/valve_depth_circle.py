import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
from collections import deque


class DepthValveEllipseDetector(Node):
    def __init__(self):
        super().__init__('depth_valve_ellipse_detector')

        self.topic_name = '/depth'
        # self.topic_name = '/zed/zed_node/point_cloud/cloud_registered'


        self.image_width = 1280
        self.image_height = 720

        self.depth_min = 0.22
        self.depth_max = 0.42

        self.latest_x = None
        self.latest_y = None
        self.latest_z = None

        self.sub = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.callback,
            10
        )

        cv2.namedWindow('depth_color', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth_mask', cv2.WINDOW_NORMAL)
        cv2.namedWindow('valve_ellipse', cv2.WINDOW_NORMAL)
        cv2.namedWindow('valve_region', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth_edge', cv2.WINDOW_NORMAL)

        self.get_logger().info('depth valve ellipse detector started')

    def callback(self, msg):
        try:
            x_offset = None
            y_offset = None
            z_offset = None

            for f in msg.fields:
                if f.name == 'x':
                    x_offset = f.offset
                elif f.name == 'y':
                    y_offset = f.offset
                elif f.name == 'z':
                    z_offset = f.offset

            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error('x/y/z field not found')
                return

            total_points = len(msg.data) // msg.point_step
            expected_points = self.image_width * self.image_height

            if total_points != expected_points:
                self.get_logger().error(
                    f'point count mismatch: total={total_points}, expected={expected_points}'
                )
                return

            x_1d = np.ndarray(
                shape=(total_points,),
                dtype=np.float32,
                buffer=msg.data,
                offset=x_offset,
                strides=(msg.point_step,)
            ).copy()

            y_1d = np.ndarray(
                shape=(total_points,),
                dtype=np.float32,
                buffer=msg.data,
                offset=y_offset,
                strides=(msg.point_step,)
            ).copy()

            z_1d = np.ndarray(
                shape=(total_points,),
                dtype=np.float32,
                buffer=msg.data,
                offset=z_offset,
                strides=(msg.point_step,)
            ).copy()

            self.latest_x = x_1d.reshape((self.image_height, self.image_width))
            self.latest_y = y_1d.reshape((self.image_height, self.image_width))
            self.latest_z = z_1d.reshape((self.image_height, self.image_width))

            self.process()

        except Exception as e:
            self.get_logger().error(f'callback failed: {e}')

    def make_depth_visual(self, depth):
        valid = np.isfinite(depth) & (depth > 0.0)
        if not np.any(valid):
            return np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        depth_vis = depth.copy()
        depth_vis[~valid] = self.depth_max
        depth_vis = np.clip(depth_vis, self.depth_min, self.depth_max)

        depth_norm = ((depth_vis - self.depth_min) / (self.depth_max - self.depth_min) * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_TURBO)
        return color

    def make_mask(self, depth):
        valid = np.isfinite(depth) & (depth > 0.0)
        mask = valid & (depth >= self.depth_min) & (depth <= self.depth_max)
        mask = (mask.astype(np.uint8) * 255)

        kernel1 = np.ones((5, 5), np.uint8)
        kernel2 = np.ones((9, 9), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel2)

        return mask

    def choose_best_contour(self, contours, img_shape):
        h, w = img_shape[:2]
        img_cx = w / 2.0
        img_cy = h / 2.0

        best_score = -1e9
        best_contour = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 3000:
                continue

            if len(cnt) < 20:
                continue

            try:
                ellipse = cv2.fitEllipse(cnt)
            except:
                continue

            (cx, cy), (ma, mi), angle = ellipse

            if ma <= 0 or mi <= 0:
                continue

            major = max(ma, mi)
            minor = min(ma, mi)
            ratio = minor / major

            if major < 80 or major > 900:
                continue

            if minor < 40:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            center_dist = np.hypot(cx - img_cx, cy - img_cy)

            score = 0.0
            score += area * 0.002
            score += ratio * 200.0
            score += circularity * 120.0
            score -= center_dist * 0.08

            if score > best_score:
                best_score = score
                best_contour = cnt

        return best_contour

    def draw_axes(self, image, ellipse):
        (cx, cy), (w, h), angle = ellipse

        a = w / 2.0
        b = h / 2.0
        theta = np.deg2rad(angle)

        major_dx = a * np.cos(theta)
        major_dy = a * np.sin(theta)

        minor_dx = -b * np.sin(theta)
        minor_dy = b * np.cos(theta)

        p1 = (int(cx - major_dx), int(cy - major_dy))
        p2 = (int(cx + major_dx), int(cy + major_dy))
        p3 = (int(cx - minor_dx), int(cy - minor_dy))
        p4 = (int(cx + minor_dx), int(cy + minor_dy))

        cv2.line(image, p1, p2, (0, 255, 0), 2)
        cv2.line(image, p3, p4, (255, 255, 0), 2)

    def find_valid_seed_near_center(self, z_map, cx, cy, max_radius=20):
        h, w = z_map.shape
        cx = int(np.clip(cx, 0, w - 1))
        cy = int(np.clip(cy, 0, h - 1))

        if np.isfinite(z_map[cy, cx]) and z_map[cy, cx] > 0:
            return cx, cy

        for r in range(1, max_radius + 1):
            x0 = max(0, cx - r)
            x1 = min(w - 1, cx + r)
            y0 = max(0, cy - r)
            y1 = min(h - 1, cy + r)

            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    if np.isfinite(z_map[y, x]) and z_map[y, x] > 0:
                        return x, y

        return None

    def point_inside_ellipse(self, x, y, ellipse, scale=1.08):
        (cx, cy), (w, h), angle = ellipse

        a = max(w * scale / 2.0, 1.0)
        b = max(h * scale / 2.0, 1.0)

        theta = np.deg2rad(angle)
        dx = x - cx
        dy = y - cy

        xr = dx * np.cos(theta) + dy * np.sin(theta)
        yr = -dx * np.sin(theta) + dy * np.cos(theta)

        value = (xr * xr) / (a * a) + (yr * yr) / (b * b)
        return value <= 1.0

    def compute_depth_gradient(self, z_map):
        valid = np.isfinite(z_map) & (z_map > 0.0)

        if not np.any(valid):
            grad = np.zeros_like(z_map, dtype=np.float32)
            edge = np.zeros_like(z_map, dtype=np.uint8)
            return grad, edge

        z = z_map.copy().astype(np.float32)
        median_z = np.median(z[valid])
        z[~valid] = median_z

        z_blur = cv2.GaussianBlur(z, (5, 5), 0)

        gx = cv2.Sobel(z_blur, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(z_blur, cv2.CV_32F, 0, 1, ksize=3)
        grad = np.sqrt(gx * gx + gy * gy)

        grad[~valid] = 0.0

        grad_threshold = 0.010
        edge = ((grad > grad_threshold) & valid).astype(np.uint8) * 255

        kernel = np.ones((3, 3), np.uint8)
        edge = cv2.dilate(edge, kernel, iterations=1)

        return grad, edge

    def make_gradient_visual(self, grad_map):
        g = grad_map.copy()
        g = np.clip(g, 0.0, 0.03)
        g = (g / 0.03 * 255.0).astype(np.uint8)
        return cv2.applyColorMap(g, cv2.COLORMAP_JET)

    def region_grow_by_gradient(self, z_map, seed_x, seed_y, ellipse, grad_map, edge_mask):
        h, w = z_map.shape

        visited = np.zeros((h, w), dtype=np.uint8)
        region = np.zeros((h, w), dtype=np.uint8)

        seed_z = z_map[seed_y, seed_x]
        if not np.isfinite(seed_z) or seed_z <= 0:
            return region

        q = deque()
        q.append((seed_x, seed_y))
        visited[seed_y, seed_x] = 1
        region[seed_y, seed_x] = 255

        neighbors = [
            (-1, -1), (0, -1), (1, -1),
            (-1,  0),          (1,  0),
            (-1,  1), (0,  1), (1,  1)
        ]

        local_step_threshold = 0.012
        grad_relax_gain = 0.35
        max_region_size = 200000
        region_count = 1

        while q:
            x, y = q.popleft()
            current_z = z_map[y, x]
            current_grad = grad_map[y, x]

            for dx, dy in neighbors:
                nx = x + dx
                ny = y + dy

                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue

                if visited[ny, nx]:
                    continue
                visited[ny, nx] = 1

                if not self.point_inside_ellipse(nx, ny, ellipse, scale=1.10):
                    continue

                nz = z_map[ny, nx]
                if not np.isfinite(nz) or nz <= 0:
                    continue

                if edge_mask[ny, nx] > 0:
                    continue

                neighbor_grad = grad_map[ny, nx]
                adaptive_step = local_step_threshold + grad_relax_gain * min(current_grad, neighbor_grad)

                if abs(nz - current_z) > adaptive_step:
                    continue

                region[ny, nx] = 255
                q.append((nx, ny))
                region_count += 1

                if region_count >= max_region_size:
                    return region

        kernel = np.ones((3, 3), np.uint8)
        region = cv2.morphologyEx(region, cv2.MORPH_CLOSE, kernel)
        region = cv2.morphologyEx(region, cv2.MORPH_OPEN, kernel)

        return region

    def keep_connected_component_from_seed(self, region, seed_x, seed_y):
        num_labels, labels = cv2.connectedComponents(region)

        if num_labels <= 1:
            return region

        if labels[seed_y, seed_x] == 0:
            return np.zeros_like(region)

        out = np.zeros_like(region)
        out[labels == labels[seed_y, seed_x]] = 255
        return out

    def extract_valve_only(self, depth_color, region_mask):
        valve_only = np.zeros_like(depth_color)
        valve_only[region_mask > 0] = depth_color[region_mask > 0]
        return valve_only

    def process(self):
        if self.latest_z is None:
            return

        x_map = self.latest_x
        y_map = self.latest_y
        z_map = self.latest_z

        depth_color = self.make_depth_visual(z_map)
        mask = self.make_mask(z_map)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        result = depth_color.copy()

        if len(contours) == 0:
            cv2.putText(result, 'no contour', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            empty = np.zeros_like(depth_color)
            self.show(depth_color, mask, result, empty, empty)
            return

        best_contour = self.choose_best_contour(contours, result.shape)
        if best_contour is None:
            cv2.putText(result, 'no valid valve contour', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            empty = np.zeros_like(depth_color)
            self.show(depth_color, mask, result, empty, empty)
            return

        cv2.drawContours(result, [best_contour], -1, (180, 180, 180), 1)

        outer_contour = cv2.convexHull(best_contour)

        if len(outer_contour) < 5:
            cv2.putText(result, 'outer contour too small', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            empty = np.zeros_like(depth_color)
            self.show(depth_color, mask, result, empty, empty)
            return

        cv2.drawContours(result, [outer_contour], -1, (255, 255, 255), 2)

        ellipse = cv2.fitEllipse(outer_contour)
        (cx, cy), (ew, eh), angle = ellipse

        cv2.ellipse(result, ellipse, (0, 0, 255), 3)
        cv2.circle(result, (int(cx), int(cy)), 5, (0, 255, 0), -1)
        self.draw_axes(result, ellipse)
        cxi = int(np.clip(round(cx), 0, self.image_width - 1))
        cyi = int(np.clip(round(cy), 0, self.image_height - 1))

        seed = self.find_valid_seed_near_center(z_map, cxi, cyi, max_radius=20)
        if seed is None:
            cv2.putText(result, 'no valid seed near center', (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
            empty = np.zeros_like(depth_color)
            self.show(depth_color, mask, result, empty, empty)
            return

        sx, sy = seed
        cv2.circle(result, (sx, sy), 6, (255, 0, 255), -1)

        grad_map, edge_mask = self.compute_depth_gradient(z_map)
        edge_mask[sy, sx] = 0
        edge_vis = self.make_gradient_visual(grad_map)

        region_mask = self.region_grow_by_gradient(
            z_map=z_map,
            seed_x=sx,
            seed_y=sy,
            ellipse=ellipse,
            grad_map=grad_map,
            edge_mask=edge_mask
        )

        region_mask = self.keep_connected_component_from_seed(region_mask, sx, sy)
        valve_only = self.extract_valve_only(depth_color, region_mask)

        region_contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(region_contours) > 0:
            biggest = max(region_contours, key=cv2.contourArea)
            cv2.drawContours(result, [biggest], -1, (0, 255, 255), 2)

        center_x3d = x_map[sy, sx]
        center_y3d = y_map[sy, sx]
        center_z3d = z_map[sy, sx]

        major = max(ew, eh)
        minor = min(ew, eh)
        region_area = int(np.count_nonzero(region_mask))

        text1 = f'center(px): ({cxi}, {cyi})'
        text2 = f'seed(px): ({sx}, {sy})  center(3d): x={center_x3d:.4f}, y={center_y3d:.4f}, z={center_z3d:.4f} m'
        text3 = f'ellipse: major={major:.1f}px, minor={minor:.1f}px, angle={angle:.1f} deg'
        text4 = f'region area: {region_area} px'

        cv2.putText(result, text1, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
        cv2.putText(result, text2, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(result, text3, (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)
        cv2.putText(result, text4, (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

        self.show(depth_color, mask, result, valve_only, edge_vis)

    def show(self, depth_color, mask, result, valve_only, edge_vis):
        depth_disp = cv2.resize(depth_color, (960, 540))
        mask_disp = cv2.resize(mask, (960, 540), interpolation=cv2.INTER_NEAREST)
        result_disp = cv2.resize(result, (960, 540))
        valve_disp = cv2.resize(valve_only, (960, 540))
        edge_vis_disp = cv2.resize(edge_vis, (960, 540))

        cv2.imshow('depth_color', depth_disp)
        cv2.imshow('depth_mask', mask_disp)
        cv2.imshow('valve_ellipse', result_disp)
        cv2.imshow('valve_region', valve_disp)
        cv2.imshow('depth_edge', edge_vis_disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthValveEllipseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()