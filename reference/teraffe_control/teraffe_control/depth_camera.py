import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2

class DepthPclViewer(Node):
    def __init__(self):
        super().__init__('depth_pcl_viewer')

        self.image_width = 1280
        self.image_height = 720

        self.topic_name = '/depth'

        self.min_depth = 0.2
        self.max_depth = 0.4

        self.clicked_x = self.image_width // 2
        self.clicked_y = self.image_height // 2
        self.latest_depth = None

        self.sub = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.callback,
            10
        )

        cv2.namedWindow('depth_pcl_z_view', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('depth_pcl_z_view', self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        if self.latest_depth is None:
            return

        h, w = self.latest_depth.shape
        win_w = 960
        win_h = 540

        img_x = int(x * w / win_w)
        img_y = int(y * h / win_h)

        img_x = np.clip(img_x, 0, w - 1)
        img_y = np.clip(img_y, 0, h - 1)

        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_x = img_x
            self.clicked_y = img_y

    def callback(self, msg):
        try:
            z_offset = None
            for f in msg.fields:
                if f.name == 'z':
                    z_offset = f.offset
                    break

            if z_offset is None:
                self.get_logger().error('z field not found')
                return

            total_points = len(msg.data) // msg.point_step
            expected_points = self.image_width * self.image_height

            if total_points != expected_points:
                self.get_logger().error(
                    f'point count mismatch: total_points={total_points}, expected={expected_points}'
                )
                return

            depth_1d = np.ndarray(
                shape=(total_points,),
                dtype=np.float32,
                buffer=msg.data,
                offset=z_offset,
                strides=(msg.point_step,)
            ).copy()

            depth = depth_1d.reshape((self.image_height, self.image_width))
            self.latest_depth = depth

        except Exception as e:
            self.get_logger().error(f'parse failed: {e}')
            return

        valid = np.isfinite(depth) & (depth > 0.0)
        if not np.any(valid):
            return

        depth_vis = depth.copy()
        depth_vis[~valid] = self.max_depth
        depth_vis = np.clip(depth_vis, self.min_depth, self.max_depth)

        depth_norm = ((depth_vis - self.min_depth) / (self.max_depth - self.min_depth) * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_JET)

        levels = 12
        depth_step = (self.max_depth - self.min_depth) / levels
        depth_quant = np.floor((depth_vis - self.min_depth) / depth_step) * depth_step + self.min_depth
        depth_quant = np.clip(depth_quant, self.min_depth, self.max_depth)
        depth_quant_norm = ((depth_quant - self.min_depth) / (self.max_depth - self.min_depth) * 255.0).astype(np.uint8)
        color_quant = cv2.applyColorMap(255 - depth_quant_norm, cv2.COLORMAP_JET)

        display = cv2.resize(color, (960, 540))
        display_quant = cv2.resize(color_quant, (960, 540), interpolation=cv2.INTER_NEAREST)

        cx = self.clicked_x
        cy = self.clicked_y
        clicked_depth = depth[cy, cx]

        center_x = self.image_width // 2
        center_y = self.image_height // 2
        center_depth = depth[center_y, center_x]

        disp_x = int(cx * 960 / self.image_width)
        disp_y = int(cy * 540 / self.image_height)
        cen_disp_x = int(center_x * 960 / self.image_width)
        cen_disp_y = int(center_y * 540 / self.image_height)

        cv2.drawMarker(display, (disp_x, disp_y), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.drawMarker(display, (cen_disp_x, cen_disp_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        text1 = f'clicked: ({cx}, {cy}) z={clicked_depth:.4f} m'
        text2 = f'center : ({center_x}, {center_y}) z={center_depth:.4f} m'
        text3 = f'range  : {self.min_depth:.2f} ~ {self.max_depth:.2f} m'

        cv2.putText(display, text1, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(display, text2, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(display, text3, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if np.isfinite(clicked_depth) and np.isfinite(center_depth):
            diff = clicked_depth - center_depth
            text4 = f'diff(clicked-center): {diff:+.4f} m'
            cv2.putText(display, text4, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow('depth_pcl_z_view', display)
        cv2.imshow('depth_pcl_z_quantized', display_quant)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthPclViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()