import math
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import PoseStamped
from tf_transformations import quaternion_from_matrix
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.image_subscription_ = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.camera_info_subscription_ = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.camera_info_ = None
        self.tf_broadcaster_ = TransformBroadcaster(self)

    def image_callback(self, msg):
        cv_img = cv_bridge.CvBridge().imgmsg_to_cv2(msg, msg.encoding)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

        if self.camera_info_ is not None:
            MARKER_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            corners,ids, _ = cv2.aruco.detectMarkers(cv_img, MARKER_DICT)
            try:
                n_markers = len(ids)
            except:
                n_markers = 0
            CAMERA_MATRIX = np.array(self.camera_info_.k).reshape((3, 3))
            DIST_COEFFS = np.array(self.camera_info_.d)

            MARKER_LENGTH = 0.04

            if n_markers > 0:
                for i in range(n_markers):
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS)

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_pos'
                    t.child_frame_id = f'aruco_{ids[i][0]}'
                    t.transform.translation.x = tvecs[i][0][0]
                    t.transform.translation.y = tvecs[i][0][1]
                    t.transform.translation.z = tvecs[i][0][2]
                    rotation_matrix = np.eye(4)
                    rotation_matrix[0:3, 0:3],_ = cv2.Rodrigues(np.array(rvecs[i][0]))
                    q = quaternion_from_matrix(rotation_matrix)
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    self.tf_broadcaster_.sendTransform(t)

    def camera_info_callback(self, msg):
        self.camera_info_ = msg

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
