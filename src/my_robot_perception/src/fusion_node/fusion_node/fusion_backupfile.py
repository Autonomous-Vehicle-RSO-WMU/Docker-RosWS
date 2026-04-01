#!/usr/bin/env python3
"""LiDAR-camera 3D detection fusion node.

Subscribes to:
  - /lidar/detections3d  (vision_msgs/Detection3DArray, LiDAR Euclidean clusters)
  - /camera/detections3d (vision_msgs/Detection3DArray, YOLO + ZED-depth 3D boxes)
  - /zed/.../camera_info (CameraInfo, for debug image projection)

Associates detections via 3D centroid Euclidean distance using the Hungarian
algorithm (scipy.optimize.linear_sum_assignment).  LiDAR geometry is kept for
the final output; camera provides the semantic class label.

Publishes:
  - /fusion/enriched_clusters (fusion_msgs/EnrichedClusterArray)
  - /fusion/debug_image       (sensor_msgs/Image  — LiDAR boxes projected onto camera)
  - /fusion/markers           (visualization_msgs/MarkerArray — 3-D RViz boxes)
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy.time
import rclpy.duration

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D
from visualization_msgs.msg import MarkerArray, Marker
from fusion_msgs.msg import EnrichedCluster, EnrichedClusterArray

import cv_bridge
from tf2_ros import Buffer, TransformListener
from message_filters import ApproximateTimeSynchronizer, Subscriber


# ---------------------------------------------------------------------------
# Colour palette indexed by class name
# ---------------------------------------------------------------------------
_CLASS_COLOURS: dict[str, tuple] = {
    '-barrel':  (0.9, 0.5, 0.0),
    '-pothole': (0.8, 0.2, 0.8),
    '-sign':    (0.2, 0.8, 0.2),
    'lidar_only': (0.6, 0.6, 0.6),
}


def _class_colour(name: str) -> tuple:
    return _CLASS_COLOURS.get(name, (0.5, 0.8, 1.0))


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class FusionNode(Node):
    """Hungarian-assignment camera-LiDAR 3D detection fusion."""

    def __init__(self) -> None:
        super().__init__('fusion_node')

        self._declare_parameters()
        self.config = self._load_config()

        self.bridge = cv_bridge.CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info: Optional[CameraInfo] = None
        self.sync_counter = 0
        self.last_sync_log = 0.0

        self._setup_publishers()
        self._setup_subscribers()

        self.get_logger().info(
            'Fusion node ready. '
            f'gate={self.config["association"]["max_centroid_distance_m"]} m  '
            f'slop={self.config["sync"]["max_time_difference_ms"]} ms'
        )

    # ------------------------------------------------------------------
    # Parameter declaration & loading
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Frames
        self.declare_parameter('frames.lidar_frame', 'os_sensor')
        self.declare_parameter('frames.camera_frame', 'zed_left_camera_optical_frame')
        self.declare_parameter('frames.base_frame', 'base_link')

        # Topics
        self.declare_parameter('topics.lidar_detections', '/lidar/detections3d')
        self.declare_parameter('topics.camera_detections3d', '/camera/detections3d')
        self.declare_parameter('topics.camera_info', '/zed/zed_node/left/camera_info')
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/image_rect_color')

        # Outputs
        self.declare_parameter('outputs.fused_enriched_clusters', '/fusion/enriched_clusters')
        self.declare_parameter('outputs.fused_markers', '/fusion/markers')
        self.declare_parameter('outputs.fused_debug_image', '/fusion/debug_image')

        # Sync
        self.declare_parameter('sync.queue_size', 10)
        self.declare_parameter('sync.max_time_difference_ms', 100)

        # Association
        self.declare_parameter('association.max_centroid_distance_m', 2.5)
        self.declare_parameter('association.association_frame', 'os_sensor')

        # Visualization
        self.declare_parameter('visualization.publish_debug_image', True)
        self.declare_parameter('visualization.min_depth_m', 0.5)
        self.declare_parameter('visualization.max_depth_m', 30.0)

    def _load_config(self) -> dict:
        return {
            'frames': {
                'lidar_frame': self.get_parameter('frames.lidar_frame').value,
                'camera_frame': self.get_parameter('frames.camera_frame').value,
                'base_frame': self.get_parameter('frames.base_frame').value,
            },
            'topics': {
                'lidar_detections': self.get_parameter('topics.lidar_detections').value,
                'camera_detections3d': self.get_parameter('topics.camera_detections3d').value,
                'camera_info': self.get_parameter('topics.camera_info').value,
                'camera_image': self.get_parameter('topics.camera_image').value,
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
                'max_centroid_distance_m': self.get_parameter('association.max_centroid_distance_m').value,
                'association_frame': self.get_parameter('association.association_frame').value,
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
        # Camera info — RELIABLE, static
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.config['topics']['camera_info'],
            self._camera_info_callback,
            10,
        )

        # LiDAR detections inherit BEST_EFFORT from the upstream sensor QoS
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config['sync']['queue_size'],
        )
        # Camera perception node publishes RELIABLE
        qos_camera = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config['sync']['queue_size'],
        )

        self.lidar_det_sub = Subscriber(
            self,
            Detection3DArray,
            self.config['topics']['lidar_detections'],
            qos_profile=qos_lidar,
        )
        self.camera_det_sub = Subscriber(
            self,
            Detection3DArray,
            self.config['topics']['camera_detections3d'],
            qos_profile=qos_camera,
        )

        slop = self.config['sync']['max_time_difference_ms'] / 1000.0
        self.sync = ApproximateTimeSynchronizer(
            [self.lidar_det_sub, self.camera_det_sub],
            queue_size=self.config['sync']['queue_size'],
            slop=slop,
        )
        self.sync.registerCallback(self._sync_callback)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_info is None:
            self.get_logger().info(
                f'Fusion node: received CameraInfo {msg.width}×{msg.height}'
            )
        self.camera_info = msg

    def _sync_callback(
        self,
        lidar_msg: Detection3DArray,
        camera_msg: Detection3DArray,
    ) -> None:
        self._throttled_sync_log(lidar_msg.header.stamp.sec)

        # --- Transform camera detections to association frame ---
        assoc_frame = self.config['association']['association_frame']
        cam_in_lidar = self._transform_detections(camera_msg, target_frame=assoc_frame)

        # --- Build enriched cluster array ---
        enriched_array = self._associate_and_build(lidar_msg, cam_in_lidar)
        self.enriched_pub.publish(enriched_array)

        # --- Markers ---
        self.markers_pub.publish(self._build_markers(enriched_array))

        # --- Debug image (project LiDAR bboxes into camera image) ---
        if self.config['visualization']['publish_debug_image']:
            debug = self._build_debug_image(enriched_array, lidar_msg.header)
            if debug is not None:
                self.debug_image_pub.publish(debug)

    # ------------------------------------------------------------------
    # Core fusion
    # ------------------------------------------------------------------
    def _transform_detections(
        self,
        det_msg: Detection3DArray,
        target_frame: str,
    ) -> List[Tuple[np.ndarray, str, float]]:
        """Return list of (centroid_xyz_in_target_frame, class_name, score)."""
        if not det_msg.detections:
            return []

        src_frame = det_msg.header.frame_id
        if not src_frame:
            src_frame = self.config['frames']['camera_frame']

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                src_frame,
                rclpy.time.Time(),           # use latest available TF
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as exc:
            self.get_logger().warning(
                f'TF lookup {src_frame} → {target_frame} failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return []

        mat = self._tf_to_matrix(tf_msg)
        result = []
        for det in det_msg.detections:
            pos = det.bbox.center.position
            p = mat @ np.array([pos.x, pos.y, pos.z, 1.0], dtype=np.float64)
            class_name = (
                det.results[0].hypothesis.class_id if det.results else 'unknown'
            )
            score = float(det.results[0].hypothesis.score) if det.results else 0.0
            result.append((p[:3], class_name, score))
        return result

    def _associate_and_build(
        self,
        lidar_msg: Detection3DArray,
        cam_dets: List[Tuple[np.ndarray, str, float]],
    ) -> EnrichedClusterArray:
        """Hungarian-match camera detections to LiDAR detections; build output."""
        lidar_dets = lidar_msg.detections
        gate = float(self.config['association']['max_centroid_distance_m'])

        # Cost matrix (N_lidar × N_cam), inf where outside gate
        N_l = len(lidar_dets)
        N_c = len(cam_dets)
        cost = np.full((N_l, N_c), np.inf) if (N_l > 0 and N_c > 0) else np.empty((N_l, 0))

        if N_l > 0 and N_c > 0:
            for i, lidar_det in enumerate(lidar_dets):
                lx = lidar_det.bbox.center.position.x
                ly = lidar_det.bbox.center.position.y
                lz = lidar_det.bbox.center.position.z
                for j, (cam_xyz, _, _) in enumerate(cam_dets):
                    dist = math.sqrt(
                        (lx - cam_xyz[0]) ** 2
                        + (ly - cam_xyz[1]) ** 2
                        + (lz - cam_xyz[2]) ** 2
                    )
                    if dist <= gate:
                        cost[i, j] = dist

        # Hungarian assignment
        matched: dict[int, int] = {}  # lidar_idx → cam_idx
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

            # Geometry from LiDAR (authoritative)
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
                cam_xyz, cam_class, cam_score = cam_dets[cam_idx]
                ec.object_class = cam_class
                ec.camera_confidence = cam_score
                ec.camera_validated = True
                ec.association_score = float(cost[i, cam_idx])
                ec.fusion_confidence = min(
                    1.0,
                    ec.lidar_confidence * 0.5 + cam_score * 0.5,
                )
                # Store projected 2D bbox if camera_info is available
                self._fill_2d_bbox(ec, lidar_det)
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
    # Helpers
    # ------------------------------------------------------------------
    def _fill_2d_bbox(self, ec: EnrichedCluster, lidar_det: Detection3D) -> None:
        """Project 8 corners of the LiDAR 3D bbox into camera image coords."""
        if self.camera_info is None:
            return
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.config['frames']['camera_frame'],
                lidar_det.header.frame_id or self.config['frames']['lidar_frame'],
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
        except Exception:
            return

        mat = self._tf_to_matrix(tf_msg)
        cx = lidar_det.bbox.center.position
        sx = lidar_det.bbox.size
        half = np.array([sx.x / 2, sx.y / 2, sx.z / 2])

        corners = []
        for sx_sign in (-1, 1):
            for sy_sign in (-1, 1):
                for sz_sign in (-1, 1):
                    c = np.array([
                        cx.x + sx_sign * half[0],
                        cx.y + sy_sign * half[1],
                        cx.z + sz_sign * half[2],
                        1.0,
                    ])
                    p_cam = mat @ c
                    if p_cam[2] <= 0.0:
                        continue
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx_k = self.camera_info.k[2]
                    cy_k = self.camera_info.k[5]
                    u = fx * p_cam[0] / p_cam[2] + cx_k
                    v = fy * p_cam[1] / p_cam[2] + cy_k
                    corners.append((u, v))

        if corners:
            us = [c[0] for c in corners]
            vs = [c[1] for c in corners]
            ec.bbox_2d_xmin = float(min(us))
            ec.bbox_2d_ymin = float(min(vs))
            ec.bbox_2d_xmax = float(max(us))
            ec.bbox_2d_ymax = float(max(vs))

    def _build_markers(self, enriched_array: EnrichedClusterArray) -> MarkerArray:
        """Build RViz wireframe cube markers for each cluster."""
        markers = MarkerArray()
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

            # Text label
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
        self, enriched_array: EnrichedClusterArray, lidar_header
    ) -> Optional[Image]:
        """
        Build a debug image showing LiDAR 3D boxes projected onto the camera
        image plane.  Green = camera-validated, red = lidar-only.
        Requires camera_info and a recent camera image.
        """
        if self.camera_info is None:
            return None
        if not enriched_array.clusters:
            return None

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.config['frames']['camera_frame'],
                lidar_header.frame_id or self.config['frames']['lidar_frame'],
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
        except Exception as exc:
            self.get_logger().debug(f'Debug image TF failed: {exc}')
            return None

        mat = self._tf_to_matrix(tf_msg)
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        W = self.camera_info.width
        H = self.camera_info.height

        canvas = np.zeros((H, W, 3), dtype=np.uint8)

        for ec in enriched_array.clusters:
            color = (0, 220, 0) if ec.camera_validated else (0, 60, 220)
            half = np.array([ec.dimensions.x / 2, ec.dimensions.y / 2, ec.dimensions.z / 2])

            # Project 8 corners of the 3D bbox (in lidar frame) into image
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
                        p_cam = mat @ corner_l
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
                cv2.rectangle(canvas, (u1, v1), (u2, v2), color, 2)
                label = ec.object_class
                if ec.camera_validated:
                    label += f' {ec.camera_confidence:.2f}'
                cv2.putText(canvas, label, (u1, max(v1 - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

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

    def _throttled_sync_log(self, stamp_sec: int) -> None:
        now = time.time()
        if now - self.last_sync_log > 5.0:
            self.last_sync_log = now
            self.get_logger().info(
                f'Fusion sync active t={stamp_sec} (count={self.sync_counter})'
            )
        self.sync_counter += 1


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
