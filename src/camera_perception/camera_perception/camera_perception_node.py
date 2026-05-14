#!/usr/bin/env python3
"""Camera perception node (2D-only).
 ---Projection based matching, favors lidar---

Subscribes to ZED left image, runs YOLOv8 inference, and publishes a
Detection2DArray with class labels and confidence scores.  No 3D lifting
is performed — the fusion node handles 3D by projecting LiDAR boxes into
image space and matching against these 2D detections by IoU.
"""

from __future__ import annotations

import os

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

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)

import cv_bridge


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
    """YOLO inference → 2D bounding boxes with class labels."""

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
            f'camera_perception_node ready (2D-only). '
            f'Device={self._device}, frame_skip={self._frame_skip}'
        )

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Frames
        self.declare_parameter('frames.camera_frame', 'zed_left_camera_optical_frame')

        # Topics
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/color/rect/image')
        self.declare_parameter('topics.detections2d_out', '/camera/detections2d')
        self.declare_parameter('topics.debug_image_out', '/camera/debug_image')

        # Detector
        self.declare_parameter('detector.enabled', True)
        self.declare_parameter('detector.model_name', 'yolov8_road_hazards.pt')
        self.declare_parameter('detector.confidence_threshold', 0.45)
        self.declare_parameter('detector.iou_threshold', 0.45)
        self.declare_parameter('detector.device', '0')
        self.declare_parameter('detector.img_size', 640)
        self.declare_parameter('detector.max_det', 50)
        self.declare_parameter('detector.frame_skip', 1)

        # Output
        self.declare_parameter('output.publish_debug_image', True)
        self.declare_parameter('output.label_font_scale', 0.7)
        self.declare_parameter('output.label_thickness', 2)

    def _load_parameters(self) -> None:
        self._camera_frame = self.get_parameter('frames.camera_frame').value

        self._image_topic = self.get_parameter('topics.camera_image').value
        self._det2d_topic = self.get_parameter('topics.detections2d_out').value
        self._debug_image_topic = self.get_parameter('topics.debug_image_out').value

        self._detector_enabled = self.get_parameter('detector.enabled').value
        self._model_name = self.get_parameter('detector.model_name').value
        self._conf_thresh = self.get_parameter('detector.confidence_threshold').value
        self._iou_thresh = self.get_parameter('detector.iou_threshold').value
        self._device = self.get_parameter('detector.device').value
        self._img_size = self.get_parameter('detector.img_size').value
        self._max_det = self.get_parameter('detector.max_det').value
        self._frame_skip = int(self.get_parameter('detector.frame_skip').value)

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
        self.det2d_pub = self.create_publisher(Detection2DArray, self._det2d_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self._debug_image_topic, 10)

    def _setup_subscribers(self) -> None:
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            sensor_qos,
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def _image_callback(self, image_msg: Image) -> None:
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

        det2d_array, debug_img = self._process(bgr, image_msg.header)
        self.det2d_pub.publish(det2d_array)

        if self._publish_debug and debug_img is not None:
            out = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            out.header = image_msg.header
            self.debug_image_pub.publish(out)

    # ------------------------------------------------------------------
    # Core processing
    # ------------------------------------------------------------------
    def _process(self, bgr: np.ndarray, header):
        """Run YOLO inference and publish 2D detections."""
        img_h, img_w = bgr.shape[:2]

        det2d_array = Detection2DArray()
        det2d_array.header = header
        det2d_array.header.frame_id = self._camera_frame

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
            return det2d_array, debug_img

        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return det2d_array, debug_img

        boxes_xyxy = result.boxes.xyxy.cpu().numpy()     # (N, 4)
        confidences = result.boxes.conf.cpu().numpy()    # (N,)
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

        for xyxy, conf, cls_id in zip(boxes_xyxy, confidences, class_ids):
            x1 = max(0, int(xyxy[0]))
            y1 = max(0, int(xyxy[1]))
            x2 = min(img_w - 1, int(xyxy[2]))
            y2 = min(img_h - 1, int(xyxy[3]))
            if x2 <= x1 or y2 <= y1:
                continue

            class_name = self.yolo.names.get(cls_id, str(cls_id))

            # --- Build Detection2D ---
            det = Detection2D()
            det.header = det2d_array.header
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            det2d_array.detections.append(det)

            # --- Debug overlay ---
            if debug_img is not None:
                self._draw_2d_box(debug_img, x1, y1, x2, y2, class_name, conf)

        return det2d_array, debug_img

    # ------------------------------------------------------------------
    # Visualisation helpers
    # ------------------------------------------------------------------
    def _draw_2d_box(
        self,
        img: np.ndarray,
        x1: int, y1: int, x2: int, y2: int,
        class_name: str,
        conf: float,
        color=(0, 220, 0),
    ) -> None:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f'{class_name} {conf:.2f}'
        cv2.putText(
            img, label, (x1, max(y1 - 6, 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            self._font_scale, color, self._font_thickness,
        )


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
