#!/usr/bin/env python3
import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch
import torchvision.transforms as transforms

from yolop_lane_ros2.msg import LaneData

# -----------------------------
# YOLOP import path
# -----------------------------
YOLOP_DIR = os.path.expanduser("~/ros2_ws/src/YOLOP")
sys.path.insert(0, YOLOP_DIR)

from lib.config import cfg
from lib.models import get_net
from lib.utils.utils import create_logger, select_device

# -----------------------------
# Preprocessing (ImageNet norm)
# -----------------------------
normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406],
    std=[0.229, 0.224, 0.225]
)
transform = transforms.Compose([transforms.ToTensor(), normalize])


def letterbox(img_bgr, new_size=640, color=(114, 114, 114)):
    h, w = img_bgr.shape[:2]
    scale = min(new_size / h, new_size / w)
    nh, nw = int(h * scale), int(w * scale)

    resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)

    pad_w = (new_size - nw) // 2
    pad_h = (new_size - nh) // 2

    out = cv2.copyMakeBorder(
        resized,
        pad_h, new_size - nh - pad_h,
        pad_w, new_size - nw - pad_w,
        cv2.BORDER_CONSTANT,
        value=color
    )
    return out, scale, pad_w, pad_h, nw, nh


def extract_lane_edges_per_row(lane_mask_u8, row_step=5):
    H, _W = lane_mask_u8.shape[:2]
    out = []
    step = max(1, int(row_step))

    for y in range(0, H, step):
        xs = np.where(lane_mask_u8[y] == 255)[0]
        if xs.size == 0:
            continue
        left_x = int(xs.min())
        right_x = int(xs.max())
        center_x = 0.5 * (left_x + right_x)
        out.append((y, left_x, right_x, float(center_x)))

    return out


def hsv_white_lane_mask(
    frame_bgr,
    *,
    white_s_max=80,
    white_v_min=160,
    close_ksize=5,
):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    white_lo = (0, 0, int(white_v_min))
    white_hi = (179, int(white_s_max), 255)
    mask = cv2.inRange(hsv, white_lo, white_hi)

    if close_ksize and close_ksize > 1:
        k = int(close_ksize)
        if k % 2 == 0:
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return mask


def apply_trapezoid_roi(
    mask_u8,
    *,
    top_y_frac=0.62,
    bot_y_frac=1.00,
    top_width_frac=0.40,
    bot_width_frac=1.05,
    center_x_frac=0.50,
):
    H, W = mask_u8.shape[:2]
    top_y = int(np.clip(top_y_frac, 0.0, 1.0) * H)
    bot_y = int(np.clip(bot_y_frac, 0.0, 1.0) * H)
    cx = int(np.clip(center_x_frac, 0.0, 1.0) * W)

    top_half = int(0.5 * np.clip(top_width_frac, 0.0, 2.0) * W)
    bot_half = int(0.5 * np.clip(bot_width_frac, 0.0, 2.0) * W)

    pts = np.array([[
        (cx - bot_half, bot_y),
        (cx + bot_half, bot_y),
        (cx + top_half, top_y),
        (cx - top_half, top_y),
    ]], dtype=np.int32)

    roi = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(roi, pts, 255)
    return cv2.bitwise_and(mask_u8, roi)


def remove_small_components(mask_u8, min_area=800):
    if min_area <= 0:
        return mask_u8
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    out = np.zeros_like(mask_u8)
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area:
            out[labels == i] = 255
    return out


def safe_bool_mask(x_u8):
    return np.where(x_u8 > 0, 255, 0).astype(np.uint8)


class YolopLaneNode(Node):
    def __init__(self):
        super().__init__("yolop_lane_node")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("weights", os.path.expanduser("~/ros2_ws/src/YOLOP/weights/End-to-end.pth"))
        self.declare_parameter("device", "cpu")
        self.declare_parameter("img_size", 640)

        self.declare_parameter("roi_y_start", 0.55)

        self.declare_parameter("trap_enable", True)
        self.declare_parameter("trap_top_y", 0.62)
        self.declare_parameter("trap_top_width", 0.40)
        self.declare_parameter("trap_bottom_width", 1.05)
        self.declare_parameter("trap_center_x", 0.50)

        self.declare_parameter("lane_thresh", 0.50)
        self.declare_parameter("max_coverage_pct", 35.0)
        self.declare_parameter("min_lane_pixels", 300)

        self.declare_parameter("cc_min_area", 900)

        self.declare_parameter("metrics_log_every", 30)
        self.declare_parameter("jitter_window", 30)

        self.declare_parameter("csv_path", os.path.expanduser("~/yolop_lane_data.csv"))
        self.declare_parameter("row_step", 5)
        self.declare_parameter("flush_every", 30)

        self.declare_parameter("save_video", True)
        self.declare_parameter("video_path", os.path.expanduser("~/yolop_overlay.mp4"))
        self.declare_parameter("video_fps", 30.0)
        self.declare_parameter("video_codec", "mp4v")
        self.declare_parameter("video_stream", "overlay")
        self.declare_parameter("overlay_alpha", 0.45)

        self.declare_parameter("hsv_enable", True)
        self.declare_parameter("hsv_mode", "safe_gate")
        self.declare_parameter("hsv_min_support_pct", 0.05)
        self.declare_parameter("hsv_white_s_max", 80)
        self.declare_parameter("hsv_white_v_min", 160)
        self.declare_parameter("hsv_close_ksize", 5)

        # -----------------------------
        # Time/history path params
        # -----------------------------
        self.declare_parameter("center_draw_thickness", 4)
        self.declare_parameter("center_curve_degree", 2)
        self.declare_parameter("center_min_rows", 6)

        self.declare_parameter("path_min_y_frac", 0.60)
        self.declare_parameter("path_margin_px", 8.0)
        self.declare_parameter("path_half_split_frac", 0.50)

        self.declare_parameter("path_history_alpha", 0.20)
        self.declare_parameter("lane_width_alpha", 0.15)
        self.declare_parameter("path_hold_frames", 6)
        self.declare_parameter("path_conf_rows_ref", 12.0)
        self.declare_parameter("path_min_width_px", 40.0)

        self.declare_parameter("path_extend_to_bottom", True)
        self.declare_parameter("path_soft_width_factor", 0.50)
        self.declare_parameter("path_min_valid_points", 3)

        # -----------------------------
        # Balanced center params
        # -----------------------------
        self.declare_parameter("center_direct_weight_bonus", 0.30)

        # -----------------------------
        # Vehicle-anchor params
        # -----------------------------
        self.declare_parameter("vehicle_anchor_enable", True)
        self.declare_parameter("vehicle_anchor_x_frac", 0.50)
        self.declare_parameter("vehicle_anchor_y_frac", 0.98)
        self.declare_parameter("vehicle_anchor_blend", 0.35)
        self.declare_parameter("vehicle_anchor_min_dy", 25)

        # -----------------------------
        # Read parameters
        # -----------------------------
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.weights = str(self.get_parameter("weights").value)
        self.device_arg = str(self.get_parameter("device").value)
        self.img_size = int(self.get_parameter("img_size").value)

        self.roi_y_start = float(self.get_parameter("roi_y_start").value)

        self.trap_enable = bool(self.get_parameter("trap_enable").value)
        self.trap_top_y = float(self.get_parameter("trap_top_y").value)
        self.trap_top_width = float(self.get_parameter("trap_top_width").value)
        self.trap_bottom_width = float(self.get_parameter("trap_bottom_width").value)
        self.trap_center_x = float(self.get_parameter("trap_center_x").value)

        self.lane_thresh = float(self.get_parameter("lane_thresh").value)
        self.max_coverage_pct = float(self.get_parameter("max_coverage_pct").value)
        self.min_lane_pixels = int(self.get_parameter("min_lane_pixels").value)

        self.cc_min_area = int(self.get_parameter("cc_min_area").value)

        self.metrics_log_every = int(self.get_parameter("metrics_log_every").value)
        self.jitter_window = int(self.get_parameter("jitter_window").value)

        self.csv_path = str(self.get_parameter("csv_path").value)
        self.row_step = int(self.get_parameter("row_step").value)
        self.flush_every = int(self.get_parameter("flush_every").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_path = str(self.get_parameter("video_path").value)
        self.video_fps = float(self.get_parameter("video_fps").value)
        self.video_codec = str(self.get_parameter("video_codec").value)
        self.video_stream = str(self.get_parameter("video_stream").value).lower()
        self.overlay_alpha = float(self.get_parameter("overlay_alpha").value)
        self._video_writer = None

        self.hsv_enable = bool(self.get_parameter("hsv_enable").value)
        self.hsv_mode = str(self.get_parameter("hsv_mode").value).lower()
        self.hsv_min_support_pct = float(self.get_parameter("hsv_min_support_pct").value)
        self.hsv_white_s_max = int(self.get_parameter("hsv_white_s_max").value)
        self.hsv_white_v_min = int(self.get_parameter("hsv_white_v_min").value)
        self.hsv_close_ksize = int(self.get_parameter("hsv_close_ksize").value)

        self.center_draw_thickness = int(self.get_parameter("center_draw_thickness").value)
        self.center_curve_degree = int(self.get_parameter("center_curve_degree").value)
        self.center_min_rows = int(self.get_parameter("center_min_rows").value)

        self.path_min_y_frac = float(self.get_parameter("path_min_y_frac").value)
        self.path_margin_px = float(self.get_parameter("path_margin_px").value)
        self.path_half_split_frac = float(self.get_parameter("path_half_split_frac").value)

        self.path_history_alpha = float(self.get_parameter("path_history_alpha").value)
        self.lane_width_alpha = float(self.get_parameter("lane_width_alpha").value)
        self.path_hold_frames = int(self.get_parameter("path_hold_frames").value)
        self.path_conf_rows_ref = float(self.get_parameter("path_conf_rows_ref").value)
        self.path_min_width_px = float(self.get_parameter("path_min_width_px").value)

        self.path_extend_to_bottom = bool(self.get_parameter("path_extend_to_bottom").value)
        self.path_soft_width_factor = float(self.get_parameter("path_soft_width_factor").value)
        self.path_min_valid_points = int(self.get_parameter("path_min_valid_points").value)

        self.center_direct_weight_bonus = float(self.get_parameter("center_direct_weight_bonus").value)

        self.vehicle_anchor_enable = bool(self.get_parameter("vehicle_anchor_enable").value)
        self.vehicle_anchor_x_frac = float(self.get_parameter("vehicle_anchor_x_frac").value)
        self.vehicle_anchor_y_frac = float(self.get_parameter("vehicle_anchor_y_frac").value)
        self.vehicle_anchor_blend = float(self.get_parameter("vehicle_anchor_blend").value)
        self.vehicle_anchor_min_dy = int(self.get_parameter("vehicle_anchor_min_dy").value)

        # -----------------------------
        # Logger + Device
        # -----------------------------
        logger, _, _ = create_logger(cfg, cfg.LOG_DIR, "ros2")
        self.torch_device = select_device(logger, self.device_arg)
        self.half = False
        self.get_logger().info(f"Using torch device: {self.torch_device}, FP16: {self.half}")

        # -----------------------------
        # Load YOLOP model
        # -----------------------------
        self.model = get_net(cfg)
        ckpt = torch.load(self.weights, map_location=self.torch_device)
        self.model.load_state_dict(ckpt["state_dict"], strict=True)
        self.model.to(self.torch_device).eval()

        dummy = torch.zeros((1, 3, self.img_size, self.img_size), device=self.torch_device)
        _ = self.model(dummy)

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.cb, qos_profile_sensor_data)
        self.pub_mask = self.create_publisher(Image, "/yolop/lane_mask", 10)
        self.pub_overlay = self.create_publisher(Image, "/yolop/overlay", 10)
        self.pub_overlay_data = self.create_publisher(LaneData, "overlay_data", 10)

        # -----------------------------
        # Metrics state
        # -----------------------------
        self.frame_idx = 0
        self.prev_center_x = None
        self.jitter_hist = deque(maxlen=self.jitter_window)
        self.coverage_hist = deque(maxlen=self.jitter_window)
        self._t_last = time.time()
        self._fps_hist = deque(maxlen=30)

        # -----------------------------
        # Time/history path state
        # -----------------------------
        self.prev_center_curve = None
        self.prev_curve_y = None
        self.prev_left_curve = None
        self.prev_right_curve = None
        self.lane_width_est = None
        self.left_conf = 0.0
        self.right_conf = 0.0
        self.frames_since_good_both = 9999
        self._last_frame_width = 640

        # -----------------------------
        # CSV output
        # -----------------------------
        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        self._csv_fh = open(self.csv_path, "w", buffering=1)
        self._csv_fh.write("frame_index,y,left_x,right_x,center_x,coverage,stamp_sec,stamp_nanosec\n")
        self.get_logger().info(f"CSV logging to: {self.csv_path}")

        if self.save_video:
            os.makedirs(os.path.dirname(self.video_path) or ".", exist_ok=True)
            self.get_logger().info(
                f"Video saving enabled: path={self.video_path}, fps={self.video_fps}, "
                f"codec={self.video_codec}, stream={self.video_stream}"
            )

        self.get_logger().info(
            f"Lane prob thresh={self.lane_thresh:.2f}, ROI y_start={self.roi_y_start:.2f}, "
            f"HSV enable={self.hsv_enable}, mode={self.hsv_mode}, hsv_min_support_pct={self.hsv_min_support_pct}, "
            f"trap_enable={self.trap_enable}, cc_min_area={self.cc_min_area}"
        )
        self.get_logger().info(
            f"Path memory: alpha={self.path_history_alpha:.2f}, width_alpha={self.lane_width_alpha:.2f}, "
            f"hold_frames={self.path_hold_frames}"
        )
        self.get_logger().info(
            f"Balanced center: direct_weight_bonus={self.center_direct_weight_bonus:.2f}"
        )
        self.get_logger().info(
            f"Vehicle anchor: enable={self.vehicle_anchor_enable}, "
            f"x_frac={self.vehicle_anchor_x_frac:.2f}, y_frac={self.vehicle_anchor_y_frac:.2f}, "
            f"blend={self.vehicle_anchor_blend:.2f}"
        )
        self.get_logger().info(f"Subscribed: {self.image_topic}")
        self.get_logger().info(
            "Publishing: /yolop/lane_mask (mono8), /yolop/overlay (bgr8), overlay_data (LaneData)"
        )

    def _ensure_video_writer(self, frame_bgr):
        if not self.save_video or self._video_writer is not None:
            return
        h, w = frame_bgr.shape[:2]
        fps = self.video_fps if self.video_fps > 0 else 30.0
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec[:4])
        writer = cv2.VideoWriter(self.video_path, fourcc, fps, (w, h), True)
        if not writer.isOpened():
            self.get_logger().error(
                f"Failed to open VideoWriter at '{self.video_path}'. Try codec 'mp4v' (mp4) or 'XVID' (avi)."
            )
            self._video_writer = None
            self.save_video = False
            return
        self._video_writer = writer
        self.get_logger().info(f"VideoWriter opened: {w}x{h} @ {fps} fps")

    def _write_video_frame(self, frame_bgr):
        if self.save_video and self._video_writer is not None:
            self._video_writer.write(frame_bgr)

    def _apply_roi(self, mask_u8, y_start_frac):
        if y_start_frac <= 0.0:
            return mask_u8
        H, W = mask_u8.shape[:2]
        y0 = int(np.clip(y_start_frac, 0.0, 1.0) * H)
        out = mask_u8.copy()
        out[:y0, :] = 0
        return out

    def _lane_mask_from_logits(self, ll_seg_out):
        C = int(ll_seg_out.shape[1])

        if C == 2:
            probs = torch.softmax(ll_seg_out, dim=1)
            lane_p = probs[:, 1, :, :]
            mask = (lane_p > self.lane_thresh).to(torch.uint8) * 255
            return mask.squeeze(0).detach().cpu().numpy().astype(np.uint8)

        if C == 1:
            lane_p = torch.sigmoid(ll_seg_out[:, 0, :, :])
            mask = (lane_p > self.lane_thresh).to(torch.uint8) * 255
            return mask.squeeze(0).detach().cpu().numpy().astype(np.uint8)

        ll_mask = torch.argmax(ll_seg_out, dim=1)
        mask_np = (ll_mask.squeeze(0).detach().cpu().numpy().astype(np.uint8)) * 255
        return mask_np

    def _publish_lane_data(self, rows):
        lane_msg = LaneData()

        if rows:
            lane_msg.left_lane_x = [float(left_x) for (_, left_x, _, _) in rows]
            lane_msg.left_lane_y = [float(y) for (y, _, _, _) in rows]

            lane_msg.right_lane_x = [float(right_x) for (_, _, right_x, _) in rows]
            lane_msg.right_lane_y = [float(y) for (y, _, _, _) in rows]

            lane_msg.center_x = [float(center_x) for (_, _, _, center_x) in rows]
            lane_msg.center_y = [float(y) for (y, _, _, _) in rows]
        else:
            lane_msg.left_lane_x = []
            lane_msg.left_lane_y = []
            lane_msg.right_lane_x = []
            lane_msg.right_lane_y = []
            lane_msg.center_x = []
            lane_msg.center_y = []

        self.pub_overlay_data.publish(lane_msg)

    def _extract_side_rows(self, lane_mask_u8, H0, W0):
        split_x = int(self.path_half_split_frac * W0)
        y_min_keep = int(self.path_min_y_frac * H0)

        left_rows = []
        right_rows = []

        for y in range(y_min_keep, H0, self.row_step):
            xs = np.where(lane_mask_u8[y] == 255)[0]
            if xs.size == 0:
                continue

            left_xs = xs[xs < split_x]
            right_xs = xs[xs >= split_x]

            if left_xs.size > 0:
                left_rows.append((y, float(left_xs.max())))

            if right_xs.size > 0:
                right_rows.append((y, float(right_xs.min())))

        return left_rows, right_rows

    def _fit_weighted_curve(self, y_vals, x_vals, degree):
        if len(y_vals) < 2:
            return None

        deg = min(degree, len(y_vals) - 1)
        if deg < 1:
            return None

        y_vals = np.asarray(y_vals, dtype=np.float32)
        x_vals = np.asarray(x_vals, dtype=np.float32)

        weights = 1.0 + 2.0 * ((y_vals - y_vals.min()) / max(1.0, y_vals.max() - y_vals.min()))

        try:
            return np.polyfit(y_vals, x_vals, deg, w=weights)
        except Exception:
            return None

    def _blend_with_history(self, y_eval, current_curve, confidence):
        if self.prev_center_curve is None or self.prev_curve_y is None:
            return current_curve

        prev_interp = np.interp(y_eval, self.prev_curve_y, self.prev_center_curve)
        alpha = confidence * self.path_history_alpha + (1.0 - confidence) * 0.08
        alpha = float(np.clip(alpha, 0.05, 0.95))
        return (1.0 - alpha) * prev_interp + alpha * current_curve

    def _update_lane_width_estimate(self, widths):
        if widths is None or len(widths) == 0:
            return

        width_now = float(np.median(widths))
        if width_now <= 1.0:
            return

        if self.lane_width_est is None:
            self.lane_width_est = width_now
        else:
            a = float(np.clip(self.lane_width_alpha, 0.0, 1.0))
            self.lane_width_est = (1.0 - a) * self.lane_width_est + a * width_now

    def _build_balanced_centerline(self, y_eval, left_curve, right_curve, left_conf, right_conf):
        y_eval = np.asarray(y_eval, dtype=np.float32)

        have_left = left_curve is not None
        have_right = right_curve is not None

        if not have_left and not have_right:
            return None

        width_est = self.lane_width_est
        if width_est is None:
            width_est = 0.35 * self._last_frame_width

        if have_left:
            left_curve = np.asarray(left_curve, dtype=np.float32)
        if have_right:
            right_curve = np.asarray(right_curve, dtype=np.float32)

        if have_left and not have_right:
            center = left_curve + 0.5 * width_est
            return center

        if have_right and not have_left:
            center = right_curve - 0.5 * width_est
            return center

        center_direct = 0.5 * (left_curve + right_curve)
        center_from_left = left_curve + 0.5 * width_est
        center_from_right = right_curve - 0.5 * width_est

        wl = float(np.clip(left_conf, 0.05, 1.0))
        wr = float(np.clip(right_conf, 0.05, 1.0))
        wd = float(np.clip(min(left_conf, right_conf) + self.center_direct_weight_bonus, 0.10, 1.0))

        denom = wl + wr + wd
        center = (wl * center_from_left + wr * center_from_right + wd * center_direct) / denom

        center = np.maximum(center, left_curve + self.path_margin_px)
        center = np.minimum(center, right_curve - self.path_margin_px)

        return center

    def _attach_path_to_vehicle(self, rows, H0, W0):
        if not rows:
            return rows, None

        anchor_x = float(np.clip(self.vehicle_anchor_x_frac, 0.0, 1.0) * W0)
        anchor_y = float(np.clip(self.vehicle_anchor_y_frac, 0.0, 1.0) * (H0 - 1))

        y_vals = np.array([float(y) for (y, _, _, _) in rows], dtype=np.float32)
        c_vals = np.array([float(cx) for (_, _, _, cx) in rows], dtype=np.float32)

        order = np.argsort(y_vals)
        y_vals = y_vals[order]
        c_vals = c_vals[order]

        first_y = float(y_vals[-1])
        first_x = float(c_vals[-1])

        if anchor_y - first_y < float(self.vehicle_anchor_min_dy):
            center_pts = np.array(
                [[int(round(cx)), int(round(y))] for y, cx in zip(y_vals, c_vals)],
                dtype=np.int32
            ).reshape((-1, 1, 2))
            return rows, center_pts

        blend = float(np.clip(self.vehicle_anchor_blend, 0.0, 1.0))
        bridge_y = np.array([
            anchor_y,
            anchor_y - 0.35 * (anchor_y - first_y),
            anchor_y - 0.70 * (anchor_y - first_y),
        ], dtype=np.float32)

        bridge_x = np.array([
            anchor_x,
            (1.0 - blend) * anchor_x + blend * first_x,
            (1.0 - 0.75 * blend) * anchor_x + (0.75 * blend) * first_x,
        ], dtype=np.float32)

        all_y = np.concatenate([bridge_y, y_vals])
        all_x = np.concatenate([bridge_x, c_vals])

        order2 = np.argsort(all_y)
        all_y = all_y[order2]
        all_x = all_x[order2]

        smoothed_rows = []
        for y, cx in zip(all_y, all_x):
            smoothed_rows.append((int(round(y)), float(cx), float(cx), float(cx)))

        center_pts = np.array(
            [[int(round(cx)), int(round(y))] for y, cx in zip(all_y, all_x)],
            dtype=np.int32
        ).reshape((-1, 1, 2))

        return smoothed_rows, center_pts

    def _hold_previous_path(self):
        if self.prev_center_curve is None or self.prev_curve_y is None:
            return [], None
        if self.frames_since_good_both > self.path_hold_frames:
            return [], None

        rows = []
        for y, cx in zip(self.prev_curve_y, self.prev_center_curve):
            rows.append((int(y), float(cx), float(cx), float(cx)))

        return rows, None

    def _build_time_smoothed_path(self, lane_mask_u8, H0, W0):
        left_rows, right_rows = self._extract_side_rows(lane_mask_u8, H0, W0)

        left_conf = min(1.0, len(left_rows) / max(1.0, self.path_conf_rows_ref))
        right_conf = min(1.0, len(right_rows) / max(1.0, self.path_conf_rows_ref))
        both_conf = min(left_conf, right_conf)

        self.left_conf = left_conf
        self.right_conf = right_conf

        left_fit = None
        right_fit = None

        if len(left_rows) >= self.center_min_rows:
            left_y = np.array([y for y, _ in left_rows], dtype=np.float32)
            left_x = np.array([x for _, x in left_rows], dtype=np.float32)
            left_fit = self._fit_weighted_curve(left_y, left_x, self.center_curve_degree)

        if len(right_rows) >= self.center_min_rows:
            right_y = np.array([y for y, _ in right_rows], dtype=np.float32)
            right_x = np.array([x for _, x in right_rows], dtype=np.float32)
            right_fit = self._fit_weighted_curve(right_y, right_x, self.center_curve_degree)

        if left_fit is None and right_fit is None:
            self.frames_since_good_both += 1
            return self._hold_previous_path()

        y_candidates = []
        if left_rows:
            y_candidates.extend([y for y, _ in left_rows])
        if right_rows:
            y_candidates.extend([y for y, _ in right_rows])

        if not y_candidates:
            self.frames_since_good_both += 1
            return self._hold_previous_path()

        y_start = int(min(y_candidates))
        y_end = H0 - 1 if self.path_extend_to_bottom else int(max(y_candidates))

        y_start = max(0, min(y_start, H0 - 1))
        y_end = max(y_start, min(y_end, H0 - 1))

        if y_end - y_start < self.row_step:
            self.frames_since_good_both += 1
            return self._hold_previous_path()

        y_eval = np.arange(y_start, y_end + 1, self.row_step, dtype=np.float32)

        left_smooth = np.polyval(left_fit, y_eval) if left_fit is not None else None
        right_smooth = np.polyval(right_fit, y_eval) if right_fit is not None else None

        if left_smooth is not None and right_smooth is not None:
            widths = right_smooth - left_smooth
            soft_min_width = max(self.path_min_width_px * self.path_soft_width_factor, 8.0)

            valid = widths > max(soft_min_width, 2.0 * self.path_margin_px + 2.0)

            if np.count_nonzero(valid) >= self.path_min_valid_points:
                y_eval = y_eval[valid]
                left_smooth = left_smooth[valid]
                right_smooth = right_smooth[valid]
                widths = widths[valid]
                self._update_lane_width_estimate(widths)
            else:
                stronger_left = left_conf >= right_conf

                if stronger_left and left_smooth is not None:
                    left_valid = np.polyval(left_fit, y_eval)
                    center_smooth = self._build_balanced_centerline(
                        y_eval, left_valid, None, left_conf, 0.0
                    )
                elif right_smooth is not None:
                    right_valid = np.polyval(right_fit, y_eval)
                    center_smooth = self._build_balanced_centerline(
                        y_eval, None, right_valid, 0.0, right_conf
                    )
                else:
                    self.frames_since_good_both += 1
                    return self._hold_previous_path()

                center_smooth = self._blend_with_history(
                    y_eval, center_smooth, max(left_conf, right_conf) * 0.6
                )

                self.prev_center_curve = center_smooth.copy()
                self.prev_curve_y = y_eval.copy()
                self.frames_since_good_both += 1

                rows = []
                for y, cx in zip(y_eval, center_smooth):
                    rows.append((int(y), float(cx), float(cx), float(cx)))

                return rows, None

        center_smooth = self._build_balanced_centerline(
            y_eval, left_smooth, right_smooth, left_conf, right_conf
        )

        if center_smooth is None or len(center_smooth) < 2:
            self.frames_since_good_both += 1
            return self._hold_previous_path()

        conf = both_conf if (left_smooth is not None and right_smooth is not None) else max(left_conf, right_conf) * 0.6
        center_smooth = self._blend_with_history(y_eval, center_smooth, conf)

        if left_smooth is not None:
            self.prev_left_curve = left_smooth.copy()
        if right_smooth is not None:
            self.prev_right_curve = right_smooth.copy()

        self.prev_center_curve = center_smooth.copy()
        self.prev_curve_y = y_eval.copy()

        if left_smooth is not None and right_smooth is not None:
            self.frames_since_good_both = 0
        else:
            self.frames_since_good_both += 1

        rows = []
        for i, y in enumerate(y_eval):
            lx = float(left_smooth[i]) if left_smooth is not None else float(center_smooth[i])
            rx = float(right_smooth[i]) if right_smooth is not None else float(center_smooth[i])
            cx = float(center_smooth[i])
            rows.append((int(y), lx, rx, cx))

        return rows, None

    @torch.no_grad()
    def cb(self, msg: Image):
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        H0, W0 = frame_bgr.shape[:2]
        self._last_frame_width = W0

        img_lb, scale, pad_w, pad_h, nw, nh = letterbox(frame_bgr, self.img_size)
        img_rgb = cv2.cvtColor(img_lb, cv2.COLOR_BGR2RGB)

        img_t = transform(img_rgb).unsqueeze(0).to(self.torch_device)
        img_t = img_t.float()

        _det_out, _da_seg_out, ll_seg_out = self.model(img_t)

        mask_np = self._lane_mask_from_logits(ll_seg_out)

        crop = mask_np[pad_h:pad_h + nh, pad_w:pad_w + nw]
        lane_orig = cv2.resize(crop, (W0, H0), interpolation=cv2.INTER_NEAREST)
        lane_orig = safe_bool_mask(lane_orig)

        lane_orig = self._apply_roi(lane_orig, self.roi_y_start)

        if self.hsv_enable:
            hsv_mask = hsv_white_lane_mask(
                frame_bgr,
                white_s_max=self.hsv_white_s_max,
                white_v_min=self.hsv_white_v_min,
                close_ksize=self.hsv_close_ksize,
            )
            hsv_mask = self._apply_roi(hsv_mask, self.roi_y_start)

            hsv_support = 100.0 * float(np.count_nonzero(hsv_mask)) / float(H0 * W0)

            if self.hsv_mode == "augment":
                lane_orig = cv2.bitwise_or(lane_orig, hsv_mask)
            elif self.hsv_mode == "gate":
                lane_orig = cv2.bitwise_and(lane_orig, hsv_mask)
            else:
                if hsv_support >= self.hsv_min_support_pct:
                    lane_gated = cv2.bitwise_and(lane_orig, hsv_mask)
                    if np.count_nonzero(lane_gated) > 0:
                        lane_orig = lane_gated

        if self.trap_enable:
            lane_orig = apply_trapezoid_roi(
                lane_orig,
                top_y_frac=self.trap_top_y,
                bot_y_frac=1.00,
                top_width_frac=self.trap_top_width,
                bot_width_frac=self.trap_bottom_width,
                center_x_frac=self.trap_center_x,
            )

        k = 5
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        lane_orig = cv2.dilate(lane_orig, kernel, iterations=1)

        lane_orig = remove_small_components(lane_orig, min_area=self.cc_min_area)

        lane_pixels = int(np.count_nonzero(lane_orig == 255))
        coverage = 100.0 * lane_pixels / float(H0 * W0)
        if coverage > self.max_coverage_pct:
            lane_orig[:, :] = 0
            lane_pixels = 0
            coverage = 0.0

        mask_msg = self.bridge.cv2_to_imgmsg(lane_orig, encoding="mono8")
        mask_msg.header = msg.header
        self.pub_mask.publish(mask_msg)

        raw_rows = []
        publish_rows = []
        center_pts = None

        if lane_pixels >= self.min_lane_pixels:
            raw_rows = extract_lane_edges_per_row(lane_orig, row_step=self.row_step)
            publish_rows, center_pts = self._build_time_smoothed_path(lane_orig, H0, W0)

            if not publish_rows:
                publish_rows = raw_rows

        if publish_rows and self.vehicle_anchor_enable:
            publish_rows, center_pts = self._attach_path_to_vehicle(publish_rows, H0, W0)
        elif publish_rows and center_pts is None:
            center_pts = np.array(
                [[int(round(cx)), int(round(y))] for (y, _, _, cx) in publish_rows],
                dtype=np.int32
            ).reshape((-1, 1, 2))

        overlay = frame_bgr.copy()
        alpha = float(np.clip(self.overlay_alpha, 0.0, 1.0))
        if lane_pixels > 0 and alpha > 0.0:
            green = overlay.copy()
            green[lane_orig == 255] = (0, 255, 0)
            overlay = cv2.addWeighted(green, alpha, overlay, 1.0 - alpha, 0.0)

        if center_pts is not None and len(center_pts) >= 2:
            cv2.polylines(
                overlay,
                [center_pts],
                isClosed=False,
                color=(0, 0, 255),
                thickness=self.center_draw_thickness
            )

        if self.vehicle_anchor_enable:
            anchor_x = int(np.clip(self.vehicle_anchor_x_frac, 0.0, 1.0) * W0)
            anchor_y = int(np.clip(self.vehicle_anchor_y_frac, 0.0, 1.0) * (H0 - 1))
            cv2.circle(overlay, (anchor_x, anchor_y), 5, (255, 0, 0), -1)

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = msg.header
        self.pub_overlay.publish(overlay_msg)

        self._publish_lane_data(publish_rows)

        if self.save_video:
            if self.video_stream == "raw":
                to_save = frame_bgr
            elif self.video_stream == "mask":
                to_save = cv2.cvtColor(lane_orig, cv2.COLOR_GRAY2BGR)
            else:
                to_save = overlay
            self._ensure_video_writer(to_save)
            self._write_video_frame(to_save)

        self.coverage_hist.append(coverage)

        center_x = None
        jitter = None
        if publish_rows:
            center_vals = [cx for (_, _, _, cx) in publish_rows]
            if center_vals:
                center_x = float(np.mean(center_vals))
                if self.prev_center_x is not None:
                    jitter = abs(center_x - self.prev_center_x)
                    self.jitter_hist.append(jitter)
                self.prev_center_x = center_x
        else:
            self.prev_center_x = None

        stamp_sec = int(msg.header.stamp.sec)
        stamp_nanosec = int(msg.header.stamp.nanosec)
        if publish_rows:
            for (y, left_x, right_x, center_x_row) in publish_rows:
                self._csv_fh.write(
                    f"{self.frame_idx},{y},{left_x:.3f},{right_x:.3f},{center_x_row:.3f},"
                    f"{coverage:.6f},{stamp_sec},{stamp_nanosec}\n"
                )
            if self.flush_every > 0 and (self.frame_idx % self.flush_every == 0):
                self._csv_fh.flush()

        t_now = time.time()
        dt = t_now - self._t_last
        self._t_last = t_now
        if dt > 0:
            self._fps_hist.append(1.0 / dt)
        fps = float(np.mean(self._fps_hist)) if len(self._fps_hist) else 0.0

        self.frame_idx += 1
        if self.metrics_log_every > 0 and (self.frame_idx % self.metrics_log_every == 0):
            cov_avg = float(np.mean(self.coverage_hist)) if len(self.coverage_hist) else coverage
            center_str = "n/a" if center_x is None else f"{center_x:.1f}px"

            if len(self.jitter_hist):
                jit_avg = float(np.mean(self.jitter_hist))
                jit_last = float(self.jitter_hist[-1])
                self.get_logger().info(
                    f"FPS: {fps:.1f} | coverage={coverage:.3f}% (avg {cov_avg:.3f}%) | "
                    f"center_x={center_str} | jitter={jit_last:.2f}px (avg {jit_avg:.2f}px) | "
                    f"left_conf={self.left_conf:.2f} | right_conf={self.right_conf:.2f} | "
                    f"frames_since_good_both={self.frames_since_good_both}"
                )
            else:
                self.get_logger().info(
                    f"FPS: {fps:.1f} | coverage={coverage:.3f}% (avg {cov_avg:.3f}%) | "
                    f"center_x={center_str} | jitter=n/a | "
                    f"left_conf={self.left_conf:.2f} | right_conf={self.right_conf:.2f} | "
                    f"frames_since_good_both={self.frames_since_good_both}"
                )


def main():
    rclpy.init()
    node = YolopLaneNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._csv_fh.flush()
            node._csv_fh.close()
        except Exception:
            pass

        try:
            if getattr(node, "_video_writer", None) is not None:
                node._video_writer.release()
                node.get_logger().info("VideoWriter released.")
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
