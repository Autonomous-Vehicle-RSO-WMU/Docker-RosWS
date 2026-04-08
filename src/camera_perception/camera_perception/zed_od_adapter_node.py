#!/usr/bin/env python3
"""ZED SDK Object Detection → Detection2DArray adapter.

Subscribes to the ZED SDK's built-in OD output (zed_msgs/ObjectsStamped)
using raw byte deserialization to avoid Python binding issues with large
fixed-size array fields (Skeleton2D/3D). Extracts 2D bounding boxes,
labels, and confidence, then publishes Detection2DArray.
"""

from __future__ import annotations

import struct

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

from zed_msgs.msg import ObjectsStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import cv_bridge


def _read_cdr_string(buf: bytes, off: int):
    """Read a CDR string (4-byte length + data + null + padding)."""
    length = struct.unpack_from('<I', buf, off)[0]
    off += 4
    s = buf[off:off + length - 1].decode('utf-8', errors='replace') if length > 0 else ''
    off += length
    # Align to 4 bytes
    off = (off + 3) & ~3
    return s, off


def _align(off: int, alignment: int) -> int:
    return (off + alignment - 1) & ~(alignment - 1)


def _parse_objects_stamped(raw: bytes):
    """Parse ObjectsStamped from raw CDR bytes, extracting only what we need.

    Returns (header_stamp_sec, header_stamp_nanosec, header_frame_id, objects_list)
    where each object in objects_list is a dict with keys:
        label, label_id, confidence, tracking_state, bbox_2d (x1,y1,x2,y2)
    """
    # Skip 4-byte CDR header (encapsulation)
    off = 4

    # --- Header ---
    # stamp.sec (int32)
    stamp_sec = struct.unpack_from('<i', raw, off)[0]
    off += 4
    # stamp.nanosec (uint32)
    stamp_nanosec = struct.unpack_from('<I', raw, off)[0]
    off += 4
    # frame_id (string)
    frame_id, off = _read_cdr_string(raw, off)

    # --- objects array ---
    # Array length (uint32)
    num_objects = struct.unpack_from('<I', raw, off)[0]
    off += 4

    objects = []
    for _ in range(num_objects):
        obj = {}

        # label (string)
        obj['label'], off = _read_cdr_string(raw, off)

        # label_id (int16)
        off = _align(off, 2)
        obj['label_id'] = struct.unpack_from('<h', raw, off)[0]
        off += 2

        # sublabel (string)
        off = _align(off, 4)
        _, off = _read_cdr_string(raw, off)

        # confidence (float32)
        off = _align(off, 4)
        obj['confidence'] = struct.unpack_from('<f', raw, off)[0]
        off += 4

        # position (float32[3])
        off = _align(off, 4)
        off += 12  # skip 3 floats

        # position_covariance (float32[6])
        off += 24  # skip 6 floats

        # velocity (float32[3])
        off += 12  # skip 3 floats

        # tracking_available (bool, 1 byte)
        off += 1

        # tracking_state (int8)
        obj['tracking_state'] = struct.unpack_from('<b', raw, off)[0]
        off += 1

        # action_state (int8)
        off += 1

        # --- bounding_box_2d: BoundingBox2Di ---
        # corners: Keypoint2Di[4], each has uint32[2]
        off = _align(off, 4)
        corners = []
        for _ in range(4):
            x = struct.unpack_from('<I', raw, off)[0]
            y = struct.unpack_from('<I', raw, off + 4)[0]
            corners.append((x, y))
            off += 8
        obj['bbox_2d'] = corners

        # --- bounding_box_3d: BoundingBox3D ---
        # corners: Keypoint3D[8], each has float32[3]
        off = _align(off, 4)
        off += 8 * 12  # skip 8 * 3 floats

        # dimensions_3d (float32[3])
        off += 12  # skip 3 floats

        # skeleton_available (bool)
        off += 1

        # body_format (int8)
        off += 1

        # --- head_bounding_box_2d: BoundingBox2Df ---
        # corners: Keypoint2Df[4], each has float32[2]
        off = _align(off, 4)
        off += 4 * 8  # skip 4 * 2 floats

        # --- head_bounding_box_3d: BoundingBox3D ---
        # corners: Keypoint3D[8], each has float32[3]
        off += 8 * 12

        # head_position (float32[3])
        off += 12

        # --- skeleton_2d: Skeleton2D ---
        # keypoints: Keypoint2Df[70], each has float32[2]
        off += 70 * 8

        # --- skeleton_3d: Skeleton3D ---
        # keypoints: Keypoint3D[70], each has float32[3]
        off += 70 * 12

        objects.append(obj)

    return stamp_sec, stamp_nanosec, frame_id, objects


class ZedOdAdapterNode(Node):
    """Thin converter: ZED ObjectsStamped → Detection2DArray."""

    def __init__(self) -> None:
        super().__init__('zed_od_adapter_node')

        # Parameters
        self.declare_parameter('topics.zed_objects', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('topics.detections2d_out', '/camera/detections2d')
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('topics.debug_image_out', '/camera/debug_image')
        self.declare_parameter('frames.camera_frame', 'zed_left_camera_optical_frame')
        self.declare_parameter('filter.min_confidence', 0.30)
        self.declare_parameter('filter.require_tracking_ok', False)
        self.declare_parameter('output.publish_debug_image', True)

        self._zed_topic = self.get_parameter('topics.zed_objects').value
        self._det2d_topic = self.get_parameter('topics.detections2d_out').value
        self._image_topic = self.get_parameter('topics.camera_image').value
        self._debug_image_topic = self.get_parameter('topics.debug_image_out').value
        self._camera_frame = self.get_parameter('frames.camera_frame').value
        self._min_conf = self.get_parameter('filter.min_confidence').value
        self._require_tracking = self.get_parameter('filter.require_tracking_ok').value
        self._publish_debug = self.get_parameter('output.publish_debug_image').value

        # Bridge and cached image for debug overlay
        self._bridge = cv_bridge.CvBridge()
        self._latest_image = None

        # Publishers
        self._det2d_pub = self.create_publisher(Detection2DArray, self._det2d_topic, 10)
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)

        # QoS for ZED topics (RELIABLE to match ZED publisher)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribe to ZED OD output (raw bytes to avoid binding issues)
        self._sub = self.create_subscription(
            ObjectsStamped,
            self._zed_topic,
            self._objects_raw_cb,
            sensor_qos,
            raw=True,
        )

        # Subscribe to ZED image for debug overlay
        if self._publish_debug:
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._image_cb,
                sensor_qos,
            )

        self.get_logger().info(
            f'zed_od_adapter ready (raw mode): {self._zed_topic} → {self._det2d_topic}'
            f' | debug_image: {self._publish_debug}'
        )

    def _image_cb(self, msg: Image) -> None:
        """Cache the latest image for debug overlay."""
        self._latest_image = msg

    def _objects_raw_cb(self, raw_msg: bytes) -> None:
        try:
            stamp_sec, stamp_nanosec, frame_id, objects = _parse_objects_stamped(raw_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse ObjectsStamped: {e}', throttle_duration_sec=5.0)
            return

        det_array = Detection2DArray()
        det_array.header.stamp.sec = stamp_sec
        det_array.header.stamp.nanosec = stamp_nanosec
        det_array.header.frame_id = self._camera_frame

        for obj in objects:
            # Confidence filter (ZED uses 1-99 scale)
            conf = obj['confidence'] / 100.0
            if conf < self._min_conf:
                continue

            # Tracking filter (optional)
            if self._require_tracking and obj['tracking_state'] != 1:
                continue

            # Extract 2D bbox from corners: TL(0), TR(1), BR(2), BL(3)
            corners = obj['bbox_2d']
            x1, y1 = corners[0]
            x2, y2 = corners[2]  # BR

            if x2 <= x1 or y2 <= y1:
                continue

            det = Detection2D()
            det.header = det_array.header
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj['label'] if obj['label'] else str(obj['label_id'])
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            det_array.detections.append(det)

        self._det2d_pub.publish(det_array)

        # Draw debug overlay (always publish so we see the camera feed)
        if self._publish_debug and self._latest_image is not None:
            try:
                bgr = self._bridge.imgmsg_to_cv2(self._latest_image, desired_encoding='bgr8')
                for det in det_array.detections:
                    cx = det.bbox.center.position.x
                    cy = det.bbox.center.position.y
                    w = det.bbox.size_x
                    h = det.bbox.size_y
                    x1, y1 = int(cx - w / 2), int(cy - h / 2)
                    x2, y2 = int(cx + w / 2), int(cy + h / 2)

                    label = det.results[0].hypothesis.class_id if det.results else '?'
                    score = det.results[0].hypothesis.score if det.results else 0.0

                    cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 220, 0), 2)
                    cv2.putText(
                        bgr, f'{label} {score:.2f}', (x1, max(y1 - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2,
                    )

                out_msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
                out_msg.header = self._latest_image.header
                self._debug_pub.publish(out_msg)
            except Exception as e:
                self.get_logger().warn(f'Debug image failed: {e}', throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedOdAdapterNode()
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
