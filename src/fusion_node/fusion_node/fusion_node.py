#!/usr/bin/env python3
"""LiDAR-camera projection-based fusion node (multi-camera).

Subscribes to:
  - /lidar/detections3d  (vision_msgs/Detection3DArray, LiDAR Euclidean clusters)
  - Per camera: detections2d, camera_info, image topics

Projects LiDAR 3D boxes into each camera's image space using TF + camera
intrinsics, then matches against camera 2D detections by 2D IoU using the
Hungarian algorithm.  LiDAR provides geometry, camera provides classification.

Publishes:
  - /fusion/enriched_clusters (fusion_msgs/EnrichedClusterArray)
  - /fusion/debug_image       (sensor_msgs/Image  — LiDAR boxes projected onto camera)
  - /fusion/markers           (visualization_msgs/MarkerArray — 3-D RViz boxes)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy.time
import rclpy.duration

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from fusion_msgs.msg import EnrichedCluster, EnrichedClusterArray

import cv_bridge
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Colour palette indexed by class name
# ---------------------------------------------------------------------------
_CLASS_COLOURS: dict[str, tuple] = {
    'barrel':   (0.9, 0.5, 0.0),
    'pothole':  (0.8, 0.2, 0.8),
    'sign':     (0.2, 0.8, 0.2),
    'lidar_only': (0.6, 0.6, 0.6),
}


def _class_colour(name: str) -> tuple:
    return _CLASS_COLOURS.get(name, (0.5, 0.8, 1.0))


# ---------------------------------------------------------------------------
# Per-camera state
# ---------------------------------------------------------------------------
@dataclass
class CameraState:
    name: str
    frame: str
    camera_info: Optional[CameraInfo] = None
    latest_image: Optional[np.ndarray] = None
    latest_dets: Optional[Detection2DArray] = None
    msg_count: int = 0


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class FusionNode(Node):
    """Projection-based 2D IoU camera-LiDAR fusion (multi-camera)."""

    def __init__(self) -> None:
        super().__init__('fusion_node')

        self.bridge = cv_bridge.CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fusion_count = 0
        self.last_fusion_log = 0.0
        self._lidar_msg_count = 0

        # Load camera configs from YAML
        self.cameras: list[CameraState] = []
        self._load_camera_configs()

        # Declare remaining parameters
        self._declare_parameters()
        self.config = self._load_config()

        self._setup_publishers()
        self._setup_subscribers()

        cam_names = [c.name for c in self.cameras]
        self.get_logger().info(
            f'Fusion node ready (multi-camera: {cam_names}). '
            f'min_iou={self.config["association"]["min_iou_threshold"]}'
        )

    # ------------------------------------------------------------------
    # Camera config loading
    # ------------------------------------------------------------------
    def _load_camera_configs(self) -> None:
        """Discover cameras from the 'cameras' YAML block."""
        # Declare and read camera names from the cameras.* namespace
        # We probe for known camera names by declaring them dynamically
        self.declare_parameter('cameras.front.frame', '')
        self.declare_parameter('cameras.front.detections2d', '')
        self.declare_parameter('cameras.front.camera_info', '')
        self.declare_parameter('cameras.front.camera_image', '')
        self.declare_parameter('cameras.rear.frame', '')
        self.declare_parameter('cameras.rear.detections2d', '')
        self.declare_parameter('cameras.rear.camera_info', '')
        self.declare_parameter('cameras.rear.camera_image', '')

        for cam_name in ['front', 'rear']:
            frame = self.get_parameter(f'cameras.{cam_name}.frame').value
            if not frame:
                continue  # camera not configured
            self.cameras.append(CameraState(
                name=cam_name,
                frame=frame,
            ))
            self.get_logger().info(f'Camera "{cam_name}" configured: frame={frame}')

    # ------------------------------------------------------------------
    # Parameter declaration & loading
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Frames
        self.declare_parameter('frames.lidar_frame', 'os_sensor')
        self.declare_parameter('frames.base_frame', 'base_link')

        # Topics
        self.declare_parameter('topics.lidar_detections', '/lidar/detections3d')

        # Outputs
        self.declare_parameter('outputs.fused_enriched_clusters', '/fusion/enriched_clusters')
        self.declare_parameter('outputs.fused_markers', '/fusion/markers')
        self.declare_parameter('outputs.fused_debug_image', '/fusion/debug_image')

        # Sync
        self.declare_parameter('sync.queue_size', 10)
        self.declare_parameter('sync.max_time_difference_ms', 500)

        # Association (2D IoU projection matching)
        self.declare_parameter('association.min_iou_threshold', 0.15)

        # Visualization
        self.declare_parameter('visualization.publish_debug_image', True)
        self.declare_parameter('visualization.min_depth_m', 0.5)
        self.declare_parameter('visualization.max_depth_m', 30.0)

    def _load_config(self) -> dict:
        return {
            'frames': {
                'lidar_frame': self.get_parameter('frames.lidar_frame').value,
                'base_frame': self.get_parameter('frames.base_frame').value,
            },
            'topics': {
                'lidar_detections': self.get_parameter('topics.lidar_detections').value,
            },
            'outputs': {
                'fused_enriched_clusters': self.get_parameter('outputs.fused_enriched_clusters').value,
                'fused_markers': self.get_parameter('outputs.fused_markers').value,
                'fused_debug_image': self.get_parameter('outputs.fused_debug_image').value,
            },
            'sync': {
                'queue_size': self.get_parameter('sync.queue_size').value,
                'max_time_difference_ms': self.get_parameter('sync.max_time_difference_ms').value,
            },
            'association': {
                'min_iou_threshold': self.get_parameter('association.min_iou_threshold').value,
            },
            'visualization': {
                'publish_debug_image': self.get_parameter('visualization.publish_debug_image').value,
                'min_depth_m': self.get_parameter('visualization.min_depth_m').value,
                'max_depth_m': self.get_parameter('visualization.max_depth_m').value,
            },
        }

    # ------------------------------------------------------------------
    # Publishers and subscribers
    # ------------------------------------------------------------------
    def _setup_publishers(self) -> None:
        self.enriched_pub = self.create_publisher(
            EnrichedClusterArray,
            self.config['outputs']['fused_enriched_clusters'],
            10,
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            self.config['outputs']['fused_markers'],
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            self.config['outputs']['fused_debug_image'],
            10,
        )

    def _setup_subscribers(self) -> None:
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_image = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # LiDAR detections
        self.create_subscription(
            Detection3DArray,
            self.config['topics']['lidar_detections'],
            self._lidar_callback,
            qos_be,
        )

        # Per-camera subscriptions
        for cam in self.cameras:
            det_topic = self.get_parameter(f'cameras.{cam.name}.detections2d').value
            info_topic = self.get_parameter(f'cameras.{cam.name}.camera_info').value
            image_topic = self.get_parameter(f'cameras.{cam.name}.camera_image').value

            self.create_subscription(
                Detection2DArray, det_topic,
                lambda msg, c=cam: self._camera_det_callback(c, msg),
                qos_rel,
            )
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, c=cam: self._camera_info_callback(c, msg),
                qos_be,
            )
            self.create_subscription(
                Image, image_topic,
                lambda msg, c=cam: self._camera_image_callback(c, msg),
                qos_image,
            )

            self.get_logger().info(
                f'Subscribed to camera "{cam.name}": '
                f'dets={det_topic}, info={info_topic}, image={image_topic}'
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _camera_info_callback(self, cam: CameraState, msg: CameraInfo) -> None:
        if cam.camera_info is None:
            self.get_logger().info(
                f'Camera "{cam.name}": received CameraInfo {msg.width}x{msg.height}'
            )
        cam.camera_info = msg

    def _camera_image_callback(self, cam: CameraState, msg: Image) -> None:
        try:
            cam.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _camera_det_callback(self, cam: CameraState, msg: Detection2DArray) -> None:
        cam.msg_count += 1
        cam.latest_dets = msg
        if cam.msg_count == 1:
            n = len(msg.detections)
            self.get_logger().info(f'Camera "{cam.name}": first detections ({n} dets)')

    def _lidar_callback(self, lidar_msg: Detection3DArray) -> None:
        """Triggered on each LiDAR detection — fuse with all cameras."""
        self._lidar_msg_count += 1
        if self._lidar_msg_count == 1:
            n = len(lidar_msg.detections)
            self.get_logger().info(f'First lidar detections received ({n} dets)')

        lidar_frame = lidar_msg.header.frame_id or self.config['frames']['lidar_frame']
        lidar_dets = lidar_msg.detections
        N_l = len(lidar_dets)
        max_dt_ms = float(self.config['sync']['max_time_difference_ms'])

        # --- Collect 2D detections from ALL cameras ---
        # Each entry: (x1, y1, x2, y2, class_name, score, cam_idx)
        all_cam_2d_boxes = []
        # Per-camera projection data: cam_idx -> (tf_mat, camera_info)
        cam_projections: dict[int, tuple] = {}

        for cam_idx, cam in enumerate(self.cameras):
            if cam.camera_info is None:
                continue

            # Get TF for this camera
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    cam.frame, lidar_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                tf_mat = self._tf_to_matrix(tf_msg)
                cam_projections[cam_idx] = (tf_mat, cam.camera_info)
            except Exception as exc:
                self.get_logger().warning(
                    f'TF lookup {lidar_frame} -> {cam.name} ({cam.frame}) failed: {exc}',
                    throttle_duration_sec=5.0,
                )
                continue

            # Extract 2D boxes from this camera's latest detections
            # Discard stale detections beyond max_time_difference_ms
            if cam.latest_dets is not None and self._is_recent(
                cam.latest_dets.header.stamp, lidar_msg.header.stamp, max_dt_ms
            ):
                for det in cam.latest_dets.detections:
                    cx = det.bbox.center.position.x
                    cy = det.bbox.center.position.y
                    half_w = det.bbox.size_x / 2.0
                    half_h = det.bbox.size_y / 2.0
                    class_name = det.results[0].hypothesis.class_id if det.results else 'unknown'
                    score = float(det.results[0].hypothesis.score) if det.results else 0.0
                    all_cam_2d_boxes.append((
                        cx - half_w, cy - half_h, cx + half_w, cy + half_h,
                        class_name, score, cam_idx,
                    ))

        self._throttled_fusion_log(lidar_msg, all_cam_2d_boxes)

        # --- Project each LiDAR 3D box into ALL camera spaces ---
        # lidar_2d_boxes[i] = list of (x1, y1, x2, y2, cam_idx) per camera that sees it
        lidar_2d_per_cam: dict[int, list] = {i: [] for i in range(N_l)}

        for cam_idx, (tf_mat, cam_info) in cam_projections.items():
            for i, det in enumerate(lidar_dets):
                box_2d = self._project_3d_box_to_2d(
                    det.bbox.center.position, det.bbox.size,
                    tf_mat, cam_info,
                )
                if box_2d is not None:
                    lidar_2d_per_cam[i].append((box_2d, cam_idx))

        # --- Build combined IoU matrix and run Hungarian assignment ---
        enriched_array = self._associate_multi_camera(
            lidar_msg, lidar_dets, lidar_2d_per_cam, all_cam_2d_boxes,
        )
        self.enriched_pub.publish(enriched_array)

        # --- Markers ---
        has_any_tf = len(cam_projections) > 0
        if has_any_tf:
            self.markers_pub.publish(self._build_markers(enriched_array))

        # --- Debug image: use front camera if available ---
        if self.config['visualization']['publish_debug_image']:
            debug = self._build_debug_image(
                enriched_array, lidar_msg.header, all_cam_2d_boxes, cam_projections,
            )
            if debug is not None:
                self.debug_image_pub.publish(debug)

    # ------------------------------------------------------------------
    # Core fusion: multi-camera 2D IoU projection matching
    # ------------------------------------------------------------------
    @staticmethod
    def _compute_iou(box_a, box_b) -> float:
        inter_x1 = max(box_a[0], box_b[0])
        inter_y1 = max(box_a[1], box_b[1])
        inter_x2 = min(box_a[2], box_b[2])
        inter_y2 = min(box_a[3], box_b[3])
        inter = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)
        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union = area_a + area_b - inter
        return inter / (union + 1e-6) if union > 0 else 0.0

    def _project_3d_box_to_2d(self, position, size, tf_mat, cam_info):
        """Project 8 corners of a LiDAR 3D AABB into a specific camera's image space.

        Returns (x1, y1, x2, y2) in pixel coords, or None if behind camera.
        """
        half = np.array([size.x / 2, size.y / 2, size.z / 2])
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx_k = cam_info.k[2]
        cy_k = cam_info.k[5]

        projected = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    corner = np.array([
                        position.x + sx * half[0],
                        position.y + sy * half[1],
                        position.z + sz * half[2],
                        1.0,
                    ])
                    p_cam = tf_mat @ corner
                    if p_cam[2] <= 0.1:
                        continue
                    u = fx * p_cam[0] / p_cam[2] + cx_k
                    v = fy * p_cam[1] / p_cam[2] + cy_k
                    projected.append((u, v))

        if len(projected) < 2:
            return None

        us = [p[0] for p in projected]
        vs = [p[1] for p in projected]
        return (min(us), min(vs), max(us), max(vs))

    def _associate_multi_camera(
        self,
        lidar_msg: Detection3DArray,
        lidar_dets: list,
        lidar_2d_per_cam: dict,
        all_cam_2d_boxes: list,
    ) -> EnrichedClusterArray:
        """Hungarian-match LiDAR projected 2D boxes to all cameras' 2D boxes by IoU.

        For each LiDAR detection, we compute IoU against every camera 2D box,
        but only when they share the same camera (same projection space).
        """
        min_iou = float(self.config['association']['min_iou_threshold'])
        N_l = len(lidar_dets)
        N_c = len(all_cam_2d_boxes)

        cost = np.full((N_l, N_c), np.inf)
        iou_matrix = np.zeros((N_l, N_c))

        if N_l > 0 and N_c > 0:
            for i in range(N_l):
                for proj_box, proj_cam_idx in lidar_2d_per_cam.get(i, []):
                    for j in range(N_c):
                        # Only compare if camera detection is from the same camera
                        if all_cam_2d_boxes[j][6] != proj_cam_idx:
                            continue
                        iou = self._compute_iou(proj_box, all_cam_2d_boxes[j][:4])
                        if iou >= min_iou:
                            cost[i, j] = 1.0 - iou
                            iou_matrix[i, j] = iou

        # Hungarian assignment
        matched: dict[int, int] = {}
        if N_l > 0 and N_c > 0:
            try:
                from scipy.optimize import linear_sum_assignment
                finite = np.where(np.isinf(cost), 1e9, cost)
                row_ind, col_ind = linear_sum_assignment(finite)
                for r, c in zip(row_ind, col_ind):
                    if not np.isinf(cost[r, c]):
                        matched[int(r)] = int(c)
            except Exception as exc:
                self.get_logger().warning(f'Hungarian assignment failed: {exc}')

        # Build output
        enriched_array = EnrichedClusterArray()
        enriched_array.header = lidar_msg.header

        for i, lidar_det in enumerate(lidar_dets):
            ec = EnrichedCluster()
            ec.header = lidar_msg.header
            ec.cluster_id = i

            pos = lidar_det.bbox.center.position
            ec.centroid.x = pos.x
            ec.centroid.y = pos.y
            ec.centroid.z = pos.z
            ec.dimensions.x = lidar_det.bbox.size.x
            ec.dimensions.y = lidar_det.bbox.size.y
            ec.dimensions.z = lidar_det.bbox.size.z
            ec.orientation.w = 1.0

            ec.lidar_confidence = float(
                lidar_det.results[0].hypothesis.score
            ) if lidar_det.results else 0.5
            ec.average_distance = math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2)

            if i in matched:
                cam_idx = matched[i]
                _, _, _, _, cam_class, cam_score, _ = all_cam_2d_boxes[cam_idx]
                ec.object_class = cam_class
                ec.camera_confidence = cam_score
                ec.camera_validated = True
                ec.association_score = float(iou_matrix[i, cam_idx])
                ec.fusion_confidence = min(
                    1.0,
                    ec.lidar_confidence * 0.5 + cam_score * 0.5,
                )
                # Store best projected 2D bbox for this match
                match_cam = all_cam_2d_boxes[cam_idx][6]
                for proj_box, proj_cam_idx in lidar_2d_per_cam.get(i, []):
                    if proj_cam_idx == match_cam:
                        ec.bbox_2d_xmin = float(proj_box[0])
                        ec.bbox_2d_ymin = float(proj_box[1])
                        ec.bbox_2d_xmax = float(proj_box[2])
                        ec.bbox_2d_ymax = float(proj_box[3])
                        break
            else:
                ec.object_class = 'lidar_only'
                ec.camera_validated = False
                ec.camera_confidence = 0.0
                ec.association_score = 0.0
                ec.fusion_confidence = ec.lidar_confidence * 0.5

            enriched_array.clusters.append(ec)

        enriched_array.total_clusters = len(enriched_array.clusters)
        enriched_array.camera_validated_count = sum(
            1 for ec in enriched_array.clusters if ec.camera_validated
        )
        enriched_array.lidar_only_count = (
            enriched_array.total_clusters - enriched_array.camera_validated_count
        )
        if enriched_array.total_clusters > 0:
            enriched_array.average_fusion_confidence = (
                sum(ec.fusion_confidence for ec in enriched_array.clusters)
                / enriched_array.total_clusters
            )
        return enriched_array

    # ------------------------------------------------------------------
    # Visualization helpers
    # ------------------------------------------------------------------
    def _build_markers(self, enriched_array: EnrichedClusterArray) -> MarkerArray:
        markers = MarkerArray()
        # Clear stale markers from previous frame
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for ec in enriched_array.clusters:
            m = Marker()
            m.header = enriched_array.header
            m.ns = 'fusion_clusters'
            m.id = ec.cluster_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = ec.centroid.x
            m.pose.position.y = ec.centroid.y
            m.pose.position.z = ec.centroid.z
            m.pose.orientation.w = 1.0
            m.scale.x = max(0.05, ec.dimensions.x)
            m.scale.y = max(0.05, ec.dimensions.y)
            m.scale.z = max(0.05, ec.dimensions.z)
            r, g, b = _class_colour(ec.object_class)
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.5 if ec.camera_validated else 0.25
            m.lifetime.sec = 1
            markers.markers.append(m)

            txt = Marker()
            txt.header = enriched_array.header
            txt.ns = 'fusion_labels'
            txt.id = ec.cluster_id + 10000
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = ec.centroid.x
            txt.pose.position.y = ec.centroid.y
            txt.pose.position.z = ec.centroid.z + (ec.dimensions.z / 2 + 0.15)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.25
            txt.color.r = r; txt.color.g = g; txt.color.b = b; txt.color.a = 1.0
            label = ec.object_class
            if ec.camera_validated:
                label += f' {ec.camera_confidence:.2f}'
            txt.text = label
            txt.lifetime.sec = 1
            markers.markers.append(txt)

        return markers

    def _build_debug_image(
        self,
        enriched_array: EnrichedClusterArray,
        lidar_header,
        all_cam_2d_boxes: list,
        cam_projections: dict,
    ) -> Optional[Image]:
        """Debug image using the first available camera with CameraInfo."""
        # Use first camera that has camera_info and an image
        cam = None
        cam_idx = None
        tf_mat = None
        for idx, c in enumerate(self.cameras):
            if c.camera_info is not None and idx in cam_projections:
                cam = c
                cam_idx = idx
                tf_mat, _ = cam_projections[idx]
                break

        if cam is None or cam.camera_info is None:
            return None

        W = cam.camera_info.width
        H = cam.camera_info.height

        if cam.latest_image is not None:
            canvas = cam.latest_image.copy()
        else:
            canvas = np.zeros((H, W, 3), dtype=np.uint8)

        fx = cam.camera_info.k[0]
        fy = cam.camera_info.k[4]
        cx = cam.camera_info.k[2]
        cy = cam.camera_info.k[5]

        # Draw camera 2D detections for this camera only (cyan)
        for box in all_cam_2d_boxes:
            if box[6] != cam_idx:
                continue
            x1, y1, x2, y2, cls, score, _ = box
            cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 0), 1)
            cv2.putText(canvas, f'{cls} {score:.2f}', (int(x1), max(int(y1) - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Draw LiDAR projected boxes
        for ec in enriched_array.clusters:
            color = (0, 220, 0) if ec.camera_validated else (0, 60, 220)
            thickness = 2 if ec.camera_validated else 1
            half = np.array([ec.dimensions.x / 2, ec.dimensions.y / 2, ec.dimensions.z / 2])

            projected = []
            for sx_sign in (-1, 1):
                for sy_sign in (-1, 1):
                    for sz_sign in (-1, 1):
                        corner_l = np.array([
                            ec.centroid.x + sx_sign * half[0],
                            ec.centroid.y + sy_sign * half[1],
                            ec.centroid.z + sz_sign * half[2],
                            1.0,
                        ])
                        p_cam = tf_mat @ corner_l
                        if p_cam[2] <= 0.1:
                            continue
                        u = int(fx * p_cam[0] / p_cam[2] + cx)
                        v = int(fy * p_cam[1] / p_cam[2] + cy)
                        if 0 <= u < W and 0 <= v < H:
                            projected.append((u, v))

            if len(projected) >= 2:
                us = [p[0] for p in projected]
                vs = [p[1] for p in projected]
                u1, v1, u2, v2 = min(us), min(vs), max(us), max(vs)
                cv2.rectangle(canvas, (u1, v1), (u2, v2), color, thickness)
                label = ec.object_class
                if ec.camera_validated:
                    label += f' {ec.camera_confidence:.2f}'
                label += f' {ec.average_distance:.1f}m'
                cv2.putText(canvas, label, (u1, max(v1 - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        out = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        out.header = lidar_header
        return out

    def _tf_to_matrix(self, tf_msg) -> np.ndarray:
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rot = self._quat_to_rot(q.x, q.y, q.z, q.w)
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = rot
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def _quat_to_rot(self, qx, qy, qz, qw) -> np.ndarray:
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz
        return np.array([
            [1.0 - 2*(yy+zz), 2*(xy-wz),       2*(xz+wy)      ],
            [2*(xy+wz),       1.0 - 2*(xx+zz), 2*(yz-wx)      ],
            [2*(xz-wy),       2*(yz+wx),        1.0 - 2*(xx+yy)],
        ], dtype=np.float64)

    @staticmethod
    def _is_recent(stamp_a, stamp_b, max_dt_ms: float) -> bool:
        """Return True if two ROS stamps are within max_dt_ms of each other."""
        t_a = stamp_a.sec + stamp_a.nanosec * 1e-9
        t_b = stamp_b.sec + stamp_b.nanosec * 1e-9
        return abs(t_a - t_b) * 1000.0 <= max_dt_ms

    def _throttled_fusion_log(self, lidar_msg, all_cam_2d_boxes) -> None:
        now = time.time()
        if now - self.last_fusion_log > 5.0:
            self.last_fusion_log = now
            n_lidar = len(lidar_msg.detections)
            # Count per camera
            cam_counts = {}
            for box in all_cam_2d_boxes:
                cam_idx = box[6]
                cam_name = self.cameras[cam_idx].name if cam_idx < len(self.cameras) else '?'
                cam_counts[cam_name] = cam_counts.get(cam_name, 0) + 1
            cam_str = ', '.join(f'{n}={c}' for n, c in cam_counts.items()) or '0 camera'
            self.get_logger().info(
                f'Fusion active: {n_lidar} lidar + [{cam_str}] dets '
                f'(total fusions={self.fusion_count})'
            )
        self.fusion_count += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
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
