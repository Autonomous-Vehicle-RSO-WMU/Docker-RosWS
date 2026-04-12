#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class VideoPub(Node):
    def __init__(self):
        super().__init__("video_pub")

        self.declare_parameter("video", "")
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("fps", 0.0)

        video = self.get_parameter("video").value
        topic = self.get_parameter("topic").value
        fps_override = float(self.get_parameter("fps").value)

        if not video:
            raise RuntimeError("Set -p video:=/path/to/cont.mkv")

        video = os.path.expanduser(video)
        self.cap = cv2.VideoCapture(video)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open video: {video}")

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps <= 1:
            fps = 30.0
        self.fps = fps_override if fps_override > 0 else fps

        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / self.fps, self.tick)

        self.get_logger().info(f"Publishing {video} to {topic} at {self.fps:.2f} FPS")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().info("Video ended, looping...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VideoPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

