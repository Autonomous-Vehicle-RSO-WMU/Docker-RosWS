#!/usr/bin/env python3
"""Camera perception node.
 ---Proception based matching, favors lidar---
 
Subscribes to ZED left image + registered point cloud, runs YOLOv8 inference,
lifts 2D detections to 3D using the ZED organized point cloud (same H×W as the
image), and publishes a Detection3DArray in the camera optical frame.
"""

from __future__ import annotations

import os
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import cv_bridge
import sensor_msgs_py.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber


# ---------------------------------------------------------------------------
# Helper: locate model file via ament share directory
# ---------------------------------------------------------------------------
def _find_model(model_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('camera_perception')
    path = os.path.join(share, 'models', model_name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'YOLO model not found at: {path}')
    return path


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class CameraPerceptionNode(Node):
    """YOLO + ZED organized point cloud → 3D bounding boxes."""

    def __init__(self) -> None:
        super().__init__('camera_perception_node')

        self._declare_parameters()
        self._load_parameters()
        self._load_yolo_model()

        self.bridge = cv_bridge.CvBridge()
        self.frame_counter = 0

        self._setup_publishers()
        self._setup_subscribers()

        self.get_logger().info(
            f'camera_perception_node ready. '
            f'Device={self._device}, frame_skip={self._frame_skip}, '
            f'pc_topic={self._pointcloud_topic}'
        )

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Frames
        self.declare_parameter('frames.camera_frame', 'zed_left_camera_optical_frame')

        # Topics
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('topics.pointcloud', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('topics.detections3d_out', '/camera/detections3d')
        self.declare_parameter('topics.debug_image_out', '/camera/debug_image')
        self.declare_parameter('topics.markers_out', '/camera/detections_markers')

        # Detector
        self.declare_parameter('detector.enabled', True)
        self.declare_parameter('detector.model_name', 'yolov8_road_hazards.pt')
        self.declare_parameter('detector.confidence_threshold', 0.45)
        self.declare_parameter('detector.iou_threshold', 0.45)
        self.declare_parameter('detector.device', '0')
        self.declare_parameter('detector.img_size', 640)
        self.declare_parameter('detector.max_det', 50)
        self.declare_parameter('detector.frame_skip', 1)

        # Point cloud 3D extraction
        self.declare_parameter('pointcloud.min_depth_m', 0.5)
        self.declare_parameter('pointcloud.max_depth_m', 12.0)
        self.declare_parameter('pointcloud.min_valid_points', 5)
        self.declare_parameter('pointcloud.roi_shrink', 0.8)

        # Output
        self.declare_parameter('output.publish_debug_image', True)
        self.declare_parameter('output.label_font_scale', 0.7)
        self.declare_parameter('output.label_thickness', 2)

    def _load_parameters(self) -> None:
        self._camera_frame = self.get_parameter('frames.camera_frame').value

        self._image_topic = self.get_parameter('topics.camera_image').value
        self._pointcloud_topic = self.get_parameter('topics.pointcloud').value
        self._det3d_topic = self.get_parameter('topics.detections3d_out').value
        self._debug_image_topic = self.get_parameter('topics.debug_image_out').value
        self._markers_topic = self.get_parameter('topics.markers_out').value

        self._detector_enabled = self.get_parameter('detector.enabled').value
        self._model_name = self.get_parameter('detector.model_name').value
        self._conf_thresh = self.get_parameter('detector.confidence_threshold').value
        self._iou_thresh = self.get_parameter('detector.iou_threshold').value
        self._device = self.get_parameter('detector.device').value
        self._img_size = self.get_parameter('detector.img_size').value
        self._max_det = self.get_parameter('detector.max_det').value
        self._frame_skip = int(self.get_parameter('detector.frame_skip').value)

        self._min_depth = self.get_parameter('pointcloud.min_depth_m').value
        self._max_depth = self.get_parameter('pointcloud.max_depth_m').value
        self._min_valid_points = int(self.get_parameter('pointcloud.min_valid_points').value)
        self._roi_shrink = self.get_parameter('pointcloud.roi_shrink').value

        self._publish_debug = self.get_parameter('output.publish_debug_image').value
        self._font_scale = self.get_parameter('output.label_font_scale').value
        self._font_thickness = int(self.get_parameter('output.label_thickness').value)

    # ------------------------------------------------------------------
    # YOLO model loading
    # ------------------------------------------------------------------
    def _load_yolo_model(self) -> None:
        if not self._detector_enabled:
            self.yolo = None
            self.get_logger().warn('Detector disabled via parameter.')
            return

        try:
            from ultralytics import YOLO
            model_path = _find_model(self._model_name)
            self.yolo = YOLO(model_path)
            # Warm-up: run once on blank image so first real frame is fast
            dummy = np.zeros((self._img_size, self._img_size, 3), dtype=np.uint8)
            self.yolo(dummy, device=self._device, imgsz=self._img_size, verbose=False)
            self.get_logger().info(
                f'YOLO loaded: {model_path}  '
                f'classes={list(self.yolo.names.values())}  '
                f'device={self._device}'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model: {exc}')
            self.yolo = None

    # ------------------------------------------------------------------
    # Publishers and subscribers
    # ------------------------------------------------------------------
    def _setup_publishers(self) -> None:
        self.det3d_pub = self.create_publisher(Detection3DArray, self._det3d_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self._debug_image_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

    def _setup_subscribers(self) -> None:
        # ZED publishes sensor data as BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = Subscriber(
            self, Image, self._image_topic, qos_profile=sensor_qos
        )
        self.pc_sub = Subscriber(
            self, PointCloud2, self._pointcloud_topic, qos_profile=sensor_qos
        )

        # 50 ms slop — ZED image and point cloud share the same timestamp
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.pc_sub],
            queue_size=5,
            slop=0.05,
        )
        self.sync.registerCallback(self._sync_callback)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _sync_callback(self, image_msg: Image, cloud_msg: PointCloud2) -> None:
        if self.yolo is None:
            return

        self.frame_counter += 1
        if self.frame_counter % self._frame_skip != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return

        det3d_array, debug_img, markers = self._process(bgr, cloud_msg, image_msg.header)
        self.det3d_pub.publish(det3d_array)
        self.markers_pub.publish(markers)

        if self._publish_debug and debug_img is not None:
            out = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            out.header = image_msg.header
            self.debug_image_pub.publish(out)

    # ------------------------------------------------------------------
    # Core processing
    # ------------------------------------------------------------------
    def _process(
        self,
        bgr: np.ndarray,
        cloud_msg: PointCloud2,
        header,
    ):
        """Run YOLO inference then lift each 2D detection to 3D via the ZED organized point cloud."""
        img_h, img_w = bgr.shape[:2]

        det3d_array = Detection3DArray()
        det3d_array.header = header
        det3d_array.header.frame_id = self._camera_frame

        markers = MarkerArray()
        debug_img = bgr.copy() if self._publish_debug else None

        # --- YOLO inference ---
        try:
            results = self.yolo(
                bgr,
                conf=self._conf_thresh,
                iou=self._iou_thresh,
                imgsz=self._img_size,
                max_det=self._max_det,
                device=self._device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().warning(f'YOLO inference failed: {exc}', throttle_duration_sec=5.0)
            return det3d_array, debug_img, markers

        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return det3d_array, debug_img, markers

        boxes_xyxy = result.boxes.xyxy.cpu().numpy()     # (N, 4)
        confidences = result.boxes.conf.cpu().numpy()    # (N,)
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

        for i, (xyxy, conf, cls_id) in enumerate(
            zip(boxes_xyxy, confidences, class_ids)
        ):
            x1, y1, x2, y2 = (
                max(0, int(xyxy[0])),
                max(0, int(xyxy[1])),
                min(img_w - 1, int(xyxy[2])),
                min(img_h - 1, int(xyxy[3])),
            )
            if x2 <= x1 or y2 <= y1:
                continue

            class_name = self.yolo.names.get(cls_id, str(cls_id))

            # --- Extract 3D bbox from organized point cloud ---
            result_3d = self._extract_3d_bbox(cloud_msg, x1, y1, x2, y2, img_w, img_h)
            if result_3d is None:
                if debug_img is not None:
                    self._draw_2d_box(debug_img, x1, y1, x2, y2, class_name, conf,
                                      z_label='no_pts', color=(0, 128, 255))
                continue

            centroid, extents = result_3d
            X, Y, Z = float(centroid[0]), float(centroid[1]), float(centroid[2])
            w3d = float(extents[0])
            h3d = float(extents[1])
            d3d = float(extents[2])

            # --- Build Detection3D ---
            det = Detection3D()
            det.header = det3d_array.header
            det.bbox.center.position.x = X
            det.bbox.center.position.y = Y
            det.bbox.center.position.z = Z
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = w3d
            det.bbox.size.y = h3d
            det.bbox.size.z = d3d

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            det3d_array.detections.append(det)

            # --- Debug overlay ---
            if debug_img is not None:
                self._draw_2d_box(debug_img, x1, y1, x2, y2, class_name, conf,
                                  z_label=f'Z={Z:.1f}m', color=(0, 220, 0))

            # --- RViz marker (sphere at 3D centroid) ---
            markers.markers.append(
                self._make_sphere_marker(i, X, Y, Z, class_name, det3d_array.header)
            )

        return det3d_array, debug_img, markers

    # ------------------------------------------------------------------
    # Point cloud helpers
    # ------------------------------------------------------------------
    def _extract_3d_bbox(
        self,
        cloud_msg: PointCloud2,
        x1: int, y1: int, x2: int, y2: int,
        img_w: int, img_h: int,
    ):
        """Extract 3D centroid and extents from the ZED organized point cloud ROI.

        The ZED organized cloud may have a different resolution to the image,
        so pixel coordinates are scaled from image space to cloud space.

        Returns (centroid_xyz, extents_xyz) as numpy arrays, or None if
        insufficient valid points are found.
        """
        pts = pc2.read_points_numpy(cloud_msg, field_names=['x', 'y', 'z'], skip_nans=False)

        # read_points_numpy may return a structured or plain float array depending on
        # the installed sensor_msgs_py version.
        if pts.dtype.names:
            pts_xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32)
        else:
            pts_xyz = pts.astype(np.float32)

        pts_xyz = pts_xyz.reshape(cloud_msg.height, cloud_msg.width, 3)

        # Scale 2D bbox from image resolution to point cloud resolution
        scale_x = cloud_msg.width / img_w
        scale_y = cloud_msg.height / img_h
        x1 = int(x1 * scale_x)
        y1 = int(y1 * scale_y)
        x2 = int(x2 * scale_x)
        y2 = int(y2 * scale_y)

        # Shrink ROI inward to avoid boundary depth artefacts
        shrink = self._roi_shrink
        dx = int((x2 - x1) * (1.0 - shrink) / 2)
        dy = int((y2 - y1) * (1.0 - shrink) / 2)
        rx1 = max(0, x1 + dx)
        ry1 = max(0, y1 + dy)
        rx2 = min(cloud_msg.width, x2 - dx)
        ry2 = min(cloud_msg.height, y2 - dy)
        if rx2 <= rx1 or ry2 <= ry1:
            rx1 = max(0, x1)
            ry1 = max(0, y1)
            rx2 = min(cloud_msg.width, x2)
            ry2 = min(cloud_msg.height, y2)

        roi = pts_xyz[ry1:ry2, rx1:rx2].reshape(-1, 3)

        # Filter: all three coordinates must be finite
        valid = roi[np.isfinite(roi).all(axis=1)]
        if len(valid) == 0:
            return None

        # Filter: depth (Z in camera optical frame) must be in configured range
        depth_mask = (valid[:, 2] >= self._min_depth) & (valid[:, 2] <= self._max_depth)
        valid = valid[depth_mask]

        if len(valid) < self._min_valid_points:
            return None

        centroid = valid.mean(axis=0)
        extents = np.clip(valid.max(axis=0) - valid.min(axis=0), 0.05, 5.0)
        return centroid, extents

    # ------------------------------------------------------------------
    # Visualisation helpers
    # ------------------------------------------------------------------
    def _draw_2d_box(
        self,
        img: np.ndarray,
        x1: int, y1: int, x2: int, y2: int,
        class_name: str,
        conf: float,
        z_label: str = '',
        color=(0, 220, 0),
    ) -> None:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f'{class_name} {conf:.2f}'
        if z_label:
            label += f' {z_label}'
        cv2.putText(
            img, label, (x1, max(y1 - 6, 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            self._font_scale, color, self._font_thickness,
        )

    def _make_sphere_marker(
        self, idx: int, X: float, Y: float, Z: float,
        class_name: str, header
    ) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'camera_detections'
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = X
        m.pose.position.y = Y
        m.pose.position.z = Z
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color.a = 0.8
        # Colour by class index
        colours = [(0.0, 1.0, 0.0), (1.0, 1.0, 0.0), (0.0, 0.5, 1.0)]
        cls_idx = list(self.yolo.names.values()).index(class_name) if class_name in self.yolo.names.values() else 0
        r, g, b = colours[cls_idx % len(colours)]
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.lifetime.sec = 1
        return m


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
