#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        # Input: Listen to Camera
        self.sub = self.create_subscription(Image, '/carla/hero/rgb_front/image', self.image_cb, 10)

        # Output: creates topic to publish stop_flag to
        self.flag_pub = self.create_publisher(Bool, '/carla/hero/stop_flag', 10)

        # Setup image translator and load YOLO model
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        # Create a folder to save detection photos
        self.save_dir = os.path.join(os.getcwd(), 'saved_images')
        os.makedirs(self.save_dir, exist_ok=True)

        self.image_count = 0

        # Define what target classe/s we're looking for
        self.target_class = 'stop sign'

    def image_cb(self, msg):
        # Convert raw ROS message into python readable image
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO model on image, ignoring anything with a less than 75% convidence level
        results = self.model.predict(img, conf=0.75)
        found = False

        # Loop through all object detected by YOLO model
        for r in results:
            for box in r.boxes:
                # Extract data from object
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                name = self.model.names[cls_id]

                # Draw box w/ text around object
                cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(img, f"{name}:{conf:.2f}", (x1, max(0,y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                # Check if current object is a stop sign
                if name == self.target_class and conf > 0.75:
                    found = True

        # If current object is a stop sign
        if found:
            # Save the current image frame to disk (for every frame stop sign object is in)
            path = os.path.join(self.save_dir, f'image_{self.image_count:05d}.jpg')
            cv2.imwrite(path, img)
            self.image_count += 1

            # Publish bool stop command
            # Publishes "True" to `/carla/hero/stop_flag`
            msg_flag = Bool()
            msg_flag.data = True
            self.flag_pub.publish(msg_flag)
            self.get_logger().info('Stop sign detected and flag published')


def main(args=None):
    rclpy.init(args=args)      # Start ROS 2
    node = VisionDetector()    # Create the node
    try:
        rclpy.spin(node)       # Keep running until keyboard interrupt
    except KeyboardInterrupt:
        pass
    node.destroy_node()        # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()
