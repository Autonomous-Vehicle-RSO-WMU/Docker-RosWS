import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from zed_msgs.msg import ObjectsStamped
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion


def rot_matrix_to_quaternion(R):
    """Shepperd's method: rotation matrix -> [x, y, z, w] quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def corners_to_bbox3d(corners):
    """
    Convert ZED 8-corner bounding box to vision_msgs BoundingBox3D.

    ZED corner layout:
         1 ------- 2
        /.        /|
       0 ------- 3 |
       | .       | |
       | 5 ......| 6
       |.        |/
       4 ------- 7

    Returns (center Pose, size Vector3).
    """
    pts = np.array([[c.kp[0], c.kp[1], c.kp[2]] for c in corners], dtype=np.float64)

    center = pts.mean(axis=0)

    # Edge vectors defining the box axes
    v_x = pts[3] - pts[0]   # width:  0 -> 3  (front-top left  -> front-top right)
    v_y = pts[1] - pts[0]   # depth:  0 -> 1  (front-top left  -> back-top  left)
    v_z = pts[4] - pts[0]   # height: 0 -> 4  (front-top left  -> front-bot left)

    sx = np.linalg.norm(v_x)
    sy = np.linalg.norm(v_y)
    sz = np.linalg.norm(v_z)

    # Safe normalisation
    x_axis = v_x / sx if sx > 1e-6 else np.array([1.0, 0.0, 0.0])
    y_axis = v_y / sy if sy > 1e-6 else np.array([0.0, 1.0, 0.0])
    z_axis = v_z / sz if sz > 1e-6 else np.array([0.0, 0.0, 1.0])

    # Re-orthogonalise to handle any floating-point drift from the SDK
    z_axis = np.cross(x_axis, y_axis)
    z_len = np.linalg.norm(z_axis)
    if z_len > 1e-6:
        z_axis /= z_len
    y_axis = np.cross(z_axis, x_axis)

    R = np.column_stack([x_axis, y_axis, z_axis])
    qx, qy, qz, qw = rot_matrix_to_quaternion(R)

    pose = Pose()
    pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    size = Vector3(x=float(sx), y=float(sy), z=float(sz))

    return pose, size


class ZedDetConverter(Node):

    def __init__(self):
        super().__init__('zed_det_converter')

        # Allow remapping via launch args
        self.declare_parameter('input_topic',  '/zed/zed_node/obj_det/objects')
        self.declare_parameter('output_topic', '/zed/detections_3d')
        # Only forward objects with tracking_state == 1 (OK) or also 2 (SEARCHING)
        self.declare_parameter('include_searching', True)

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.include_searching = self.get_parameter('include_searching').value

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            ObjectsStamped,
            in_topic,
            self.objects_callback,
            best_effort_qos,
        )

        self.pub = self.create_publisher(
            Detection3DArray,
            out_topic,
            10,
        )

        self.get_logger().info(
            f'ZED→Detection3DArray converter ready\n'
            f'  Subscribing : {in_topic}\n'
            f'  Publishing  : {out_topic}'
        )

    def objects_callback(self, msg: ObjectsStamped):
        out = Detection3DArray()
        out.header = msg.header

        for obj in msg.objects:
            # tracking_state: 0=OFF, 1=OK, 2=SEARCHING, 3=TERMINATE
            if obj.tracking_state == 0 or obj.tracking_state == 3:
                continue
            if obj.tracking_state == 2 and not self.include_searching:
                continue

            corners = obj.bounding_box_3d.corners
            if len(corners) != 8:
                continue

            try:
                center_pose, bbox_size = corners_to_bbox3d(corners)
            except Exception as e:
                self.get_logger().warn(f'Corner conversion failed for obj {obj.label}: {e}')
                continue

            det = Detection3D()
            det.header = msg.header

            det.bbox = BoundingBox3D()
            det.bbox.center = center_pose
            det.bbox.size   = bbox_size

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.label
            hyp.hypothesis.score    = float(obj.confidence) / 100.0
            # Place centroid pose inside the hypothesis too (useful for some fusion nodes)
            hyp.pose.pose = center_pose
            det.results.append(hyp)

            # Tracking ID as string so downstream nodes can associate tracks
            det.id = str(obj.label_id)

            out.detections.append(det)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ZedDetConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
