import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf_conversions
import tf2_ros
from tf2_msgs.msg import TFMessage

import os
import yaml
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from geometry_msgs.msg import Transform, TransformStamped


class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector')
        self.subscriber = rospy.Subscriber('image_publisher', Image, self.callback)
        self.publisher = rospy.Publisher('tf', TFMessage, queue_size=10)
        self.bridge = CvBridge()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        self.camera_matrix = None
        self.distortion_matrix = None        
        self.marker_length = 5

        self.get_calibration_from_file()

        rospy.spin()
    
    def callback(self, image_message):
        image = self.bridge.imgmsg_to_cv2(image_message)
        gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=gray_image,
                                                     dictionary=self.aruco_dict,
                                                     parameters=self.aruco_params)

        rotation_vectors, translation_vectors, object_points = aruco.estimatePoseSingleMarkers(
            corners=corners,
            markerLength=self.marker_length,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.distortion_matrix
        )

        if ids is None:
            return

        broadcaster = tf2_ros.TransformBroadcaster()
        tf_message_list = TFMessage()

        for rotation_vector, translation_vector, _id in zip(rotation_vectors[:, 0, :], translation_vectors[:, 0, :], ids[:, 0]):
            message = TransformStamped()
            
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'map'
            message.child_frame_id = f'marker_{_id}'
            
            transform = Transform()
            quaternion = tf_conversions.transformations.quaternion_from_euler(*rotation_vector)
            
            transform.translation.x, transform.translation.y, transform.translation.z = translation_vector
            transform.translation.z /= 4
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w = quaternion

            message.transform = transform
            
            tf_message_list.transforms.append(message)
            broadcaster.sendTransform(message)

        self.publisher.publish(tf_message_list)
        print(f'id: {_id}')
        rospy.loginfo(message)

    def get_calibration_from_file(self, calibration_path: str = './src/aruco_project/scripts/calibration.yaml'):
        print(os.listdir())
        if calibration_path:
            with open(calibration_path) as file:
                calibration_file_content = yaml.safe_load(file)
            print('Loaded calibration configuration')
        else:
            raise NoCalibrationFoundException()

        self.camera_matrix = np.array(calibration_file_content.get('camera_matrix'))
        self.distortion_matrix = np.array(calibration_file_content.get('distortion_matrix'))

class NoCalibrationFoundException(Exception):
    pass


if __name__ == '__main__':
    ArucoDetector()