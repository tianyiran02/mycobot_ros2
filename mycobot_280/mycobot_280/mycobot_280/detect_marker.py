import cv2
import numpy as np
import rclpy
import tf_transformations
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion


class ImageConverter(Node):
    def __init__(self):
        super().__init__("detect_marker")
        self.br = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.aruo_params = cv2.aruco.DetectorParameters_create()
        calibrationParams = cv2.FileStorage(
            "calibrationFileName.xml", cv2.FILE_STORAGE_READ
        )
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
        self.camera_matrix = None
        # subscriber, listen wether has img come in.
        self.image_sub = self.create_subscription(
            msg_type=Image,
            topic="camera/image",
            callback=self.callback,
            qos_profile=1
        )

    def callback(self, data):
        """Callback function.

        Process image with OpenCV, detect Mark to get the pose. Then acccording the
        pose to transforming.
        """
        try:
            # trans `rgb` to `gbr` for opencv.
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        size = cv_image.shape
        focal_length = size[1]
        center = [size[1] / 2, size[0] / 2]
        if self.camera_matrix is None:
            # calc the camera matrix, if don't have.
            self.camera_matrix = np.array(
                [
                    [focal_length, 0, center[0]],
                    [0, focal_length, center[1]],
                    [0, 0, 1],
                ],
                dtype=np.float32,
            )
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # detect aruco marker.
        ret = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruo_params)
        corners, ids = ret[0], ret[1]
        # process marker data.
        if len(corners) > 0:
            if ids is not None:
                # print('corners:', corners, 'ids:', ids)

                # detect marker pose.
                # argument:
                #   marker corners
                #   marker size (meter)
                ret = cv2.aruco.estimatePoseSingleMarkers(
                    corners, 0.024, self.camera_matrix, self.dist_coeffs
                )
                (rvec, tvec) = (ret[0], ret[1])
                (rvec - tvec).any()

                print("rvec:", rvec, "tvec:", tvec)

                # just select first one detected marker.
                for i in range(rvec.shape[0]):
                    cv2.aruco.drawDetectedMarkers(cv_image, corners)
                    cv2.aruco.drawAxis(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec[i, :, :],
                        tvec[i, :, :],
                        0.03,
                    )

                xyz = tvec[0, 0, :]
                xyz = [-1.0 * (xyz[0] - 0.045), xyz[1], -1.0 * (xyz[2] - 0.03)]

                # get quaternion for ros.
                euler = rvec[0, 0, :]
                tf_change = tf_transformations.quaternion_from_euler(
                    euler[0], euler[1], euler[2]
                )
                print("tf_change:", tf_change)

                # trans pose according [joint1]
                trans_stamp = TransformStamped()
                trans_stamp.header._stamp = self.get_clock().now().to_msg()
                trans_stamp.header.frame_id = "joint6_flange"
                trans_stamp.child_frame_id = "basic_shapes"
                vector_xyz = Vector3()
                vector_rot = Quaternion()
                vector_xyz.x = xyz[0]
                vector_xyz.y = xyz[1]
                vector_xyz.z = xyz[2]
                vector_rot.x = tf_change[0]
                vector_rot.y = tf_change[1]
                vector_rot.z = tf_change[2]
                vector_rot.w = tf_change[3]
                trans_stamp.transform.translation = vector_xyz
                trans_stamp.transform.rotation = vector_rot
                self.br.sendTransform(trans_stamp)

        # # [x, y, z, -172, 3, -46.8]
        # height, width = cv_image.shape[:2]

        # # Define resize ratio
        # ratio = 0.25

        # # Compute new dimensions
        # new_height = int(height * ratio)
        # new_width = int(width * ratio)

        # # Resize image
        # resized_img = cv2.resize(cv_image, (new_width, new_height))

        # # Display resized image
        # cv2.imshow('Marker Detection Image', resized_img)

        # cv2.waitKey(3)
        try:
            pass
        except CvBridgeError as e:
            print(e)


def main(args=None):
    try:
        rclpy.init(args=args)
        print("Starting cv_bridge_test node")
        i = ImageConverter()
        rclpy.spin(i)
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
