import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time


class ArucoToTF(Node):
    def __init__(self):
        super().__init__('aruco_zed_tf_publisher')

        self.declare_parameter('marker_length', 0.24)
        self.declare_parameter('marker_id', 7)
        self.declare_parameter('camera_frame', 'zed_left_camera_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.marker_pub = self.create_publisher(Marker, 'aruco_marker_labels', 10)

        self.marker_length = self.get_parameter('marker_length').value
        self.marker_id = self.get_parameter('marker_id').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.bridge = CvBridge()
        self.K = None
        self.dist_coeffs = np.zeros((5, 1))
        self.camera_info_received = False
        self.transform_published = False

        self.cam_info_sub = self.create_subscription(CameraInfo, '/zed/zed_node/rgb/camera_info', self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)

        # self.cam_info_sub = self.create_subscription(CameraInfo, '/zed/zed_node/left_raw/camera_info', self.info_callback, 10)
        # self.image_sub = self.create_subscription(Image, '/zed/zed_node/left_raw/image_rect_color', self.image_callback, 10)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        # self.aruco_params = aruco.DetectorParameters()
        # Handle OpenCV ArUco API differences
        if hasattr(aruco, "DetectorParameters"):
            self.aruco_params = aruco.DetectorParameters()
        else:
            self.aruco_params = aruco.DetectorParameters_create()

        #old
        # Known marker pose w.r.t. base_link
        # self.marker_pos = np.array([9.327972657047212e-05, 0.6821631193161011, 0.007165226619690657])
        # #self.marker_quat = np.array([0.9998024106025696, 0.018966037780046463, 0.0023459831718355417, 0.005471593700349331])
        # self.marker_quat = np.array([ 0.00547159, -0.00234598,  0.01896604, -0.9998024]) #rotation in x

        #new
        self.marker_pos = np.array([0.0013297745026648045, 0.6970388889312744, 0.2231410026550293])
        self.marker_quat = np.array([0.005151091143488884 , 0.02410433255136013, 0.7298352122306824, -0.6831761002540588])
        self.get_logger().info("Script is running!")

    def info_callback(self, msg):
        if self.camera_info_received:
            return
        self.get_logger().info("Camera info received.")
        self.K = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d).reshape(-1, 1)
        self.camera_info_received = True

    def image_callback(self, msg):
        if self.K is None or self.transform_published:
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            image = aruco.drawDetectedMarkers(image, corners, ids)
            cv2.imshow("Aruco Detection", image)
            cv2.waitKey(1)

        if ids is not None and self.marker_id in ids:
            index = list(ids.flatten()).index(self.marker_id)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.K, self.dist_coeffs)

            rvec = rvecs[index][0]
            tvec = tvecs[index][0]

            # Marker → Camera (optical)
            R_marker_to_cam, _ = cv2.Rodrigues(rvec)
            T_marker_to_cam = np.eye(4)
            T_marker_to_cam[:3, :3] = R_marker_to_cam
            T_marker_to_cam[:3, 3] = tvec

            # Invert: Camera → Marker
            T_cam_to_marker = np.linalg.inv(T_marker_to_cam)

            # Known: Marker → Base
            R_marker_base = R.from_quat(self.marker_quat).as_matrix()
            T_marker_base = np.eye(4)
            T_marker_base[:3, :3] = R_marker_base
            T_marker_base[:3, 3] = self.marker_pos

            # Compute: Camera → Base
            T_cam_to_base = T_marker_base @ T_cam_to_marker

            self.get_logger().info("Transform matrix: Camera (optical) w.r.t. Base")
            for row in T_cam_to_base:
                self.get_logger().info(f"{row}")

            # Extract translation and quaternion
            trans = T_cam_to_base[:3, 3]
            rot = R.from_matrix(T_cam_to_base[:3, :3]).as_quat()

            # Publish camera TF
            tf_msg = TransformStamped()
            #tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.stamp = Time()
            tf_msg.header.frame_id = self.base_frame
            tf_msg.child_frame_id = self.camera_frame
            tf_msg.transform.translation.x = float(trans[0])
            tf_msg.transform.translation.y = float(trans[1])
            tf_msg.transform.translation.z = float(trans[2])
            tf_msg.transform.rotation.x = float(rot[0])
            tf_msg.transform.rotation.y = float(rot[1])
            tf_msg.transform.rotation.z = float(rot[2])
            tf_msg.transform.rotation.w = float(rot[3])
            self.tf_broadcaster.sendTransform(tf_msg)

            self.get_logger().info(f"Published TF: {self.base_frame} → {self.camera_frame}")

            # Publish marker TF in camera optical frame
            marker_tf = TransformStamped()
            #marker_tf.header.stamp = self.get_clock().now().to_msg()
            marker_tf.header.stamp = Time()
            marker_tf.header.frame_id = self.camera_frame
            marker_tf.child_frame_id = f"aruco_marker_{self.marker_id}"
            marker_tf.transform.translation.x = float(tvec[0])
            marker_tf.transform.translation.y = float(tvec[1])
            marker_tf.transform.translation.z = float(tvec[2])
            marker_quat = R.from_matrix(R_marker_to_cam).as_quat()
            marker_tf.transform.rotation.x = float(marker_quat[0])
            marker_tf.transform.rotation.y = float(marker_quat[1])
            marker_tf.transform.rotation.z = float(marker_quat[2])
            marker_tf.transform.rotation.w = float(marker_quat[3])
            self.tf_broadcaster.sendTransform(marker_tf)

            # Publish known marker pose w.r.t. base
            known_marker_tf = TransformStamped()
            #known_marker_tf.header.stamp = self.get_clock().now().to_msg()
            known_marker_tf.header.stamp = Time()
            known_marker_tf.header.frame_id = self.base_frame
            known_marker_tf.child_frame_id = f"known_aruco_marker_{self.marker_id}"
            known_marker_tf.transform.translation.x = float(self.marker_pos[0])
            known_marker_tf.transform.translation.y = float(self.marker_pos[1])
            known_marker_tf.transform.translation.z = float(self.marker_pos[2])
            known_marker_tf.transform.rotation.x = float(self.marker_quat[0])
            known_marker_tf.transform.rotation.y = float(self.marker_quat[1])
            known_marker_tf.transform.rotation.z = float(self.marker_quat[2])
            known_marker_tf.transform.rotation.w = float(self.marker_quat[3])
            self.tf_broadcaster.sendTransform(known_marker_tf)

            # Marker label
            label = Marker()
            #label.header.stamp = self.get_clock().now().to_msg()
            label.header.stamp = Time()
            label.header.frame_id = f"aruco_marker_{self.marker_id}"
            label.ns = "aruco_labels"
            label.id = self.marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.z = 0.1
            label.pose.orientation.w = 1.0
            label.scale.z = 0.1
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.0
            label.color.a = 1.0
            label.text = f"Marker {self.marker_id}"
            self.marker_pub.publish(label)

            self.transform_published = True
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
