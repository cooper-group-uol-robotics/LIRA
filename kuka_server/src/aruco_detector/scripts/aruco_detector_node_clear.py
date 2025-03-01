#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aruco_detector.msg import ArucoPose
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import numpy as np
import tf2_ros
import tf.transformations as tft


class ArucoDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.marker_pubs = {}
        # Publisher 
        self.marker_pub = rospy.Publisher('/aruco_pose', ArucoPose, queue_size=10)
        
        # Subscriber 
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Pre-defined camera calibration values 
        # for the resolution 1280 * 720
        self.camera_matrix = np.array([[919.8290405273438, 0.0, 654.5701904296875], [0.0, 918.3110961914062, 377.02618408203125], [0.0, 0.0, 1.0]], dtype=np.float32)
        # for the resolution 640 * 480
        # self.camera_matrix = np.array([[613.2193603515625, 0, 329.71343994140625], [0, 612.2073974609375, 251.35079956054688], [0, 0, 1]], dtype=np.float32)
        self.camera_distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.buffer_size = 10
        self.detection_buffer = []
        self.detection_buffers = {}
        self.last_detections = []

        # meters

        self.marker_length = 0.075 # printing tag
        # self.marker_length = 0.142 # printing tag

        # initialize tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Timer(rospy.Duration(0.1), self.publish_camera_to_link_ee_tf)

    def publish_camera_to_link_ee_tf(self, event):
        # print("Broadcasting static TF...")
        t = geometry_msgs.msg.TransformStamped()
        # headers
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "kmriiwa_link_ee"
        t.child_frame_id = "camera_color_frame"
        
        # Adjust based on E2H calibration results
        t.transform.translation.x = 0.03065913
        t.transform.translation.y = -0.07453462
        t.transform.translation.z = 0.10616843

        # quaternion = [-0.0001976, 0.7275608, -0.6859152, 0.013249]

        # Fill in the quaternion
        t.transform.rotation.x = -0.0001976
        t.transform.rotation.y = 0.7275608
        t.transform.rotation.z = -0.6859152
        t.transform.rotation.w = 0.013249

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

    def broadcast_tf(self, ids, tvec, quaternion):
        # Create a TransformStamped message
        transform = geometry_msgs.msg.TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_color_optical_frame"  # Parent frame
        transform.child_frame_id = "aruco_marker_" + str(ids)

        transform.transform.translation.x = tvec.x
        transform.transform.translation.y = tvec.y
        transform.transform.translation.z = tvec.z
        print("id is", ids)
        print('trans + quat',transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        # print('quaternion',quaternion)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(transform)

    def filter_detections(self, marker_id):
        if marker_id not in self.detection_buffers or len(self.detection_buffers[marker_id]) < self.buffer_size:
            return None
        # if len(self.detection_buffer) < self.buffer_size:
        #     return None

        # Compute mean pose from buffered detections
        mean_pose = Pose()
        for detection in self.detection_buffers[marker_id]:
            mean_pose.position.x += detection.pose.position.x
            mean_pose.position.y += detection.pose.position.y
            mean_pose.position.z += detection.pose.position.z
            mean_pose.orientation.x += detection.pose.orientation.x
            mean_pose.orientation.y += detection.pose.orientation.y
            mean_pose.orientation.z += detection.pose.orientation.z
            mean_pose.orientation.w += detection.pose.orientation.w

        mean_pose.position.x /= self.buffer_size
        mean_pose.position.y /= self.buffer_size
        mean_pose.position.z /= self.buffer_size
        mean_pose.orientation.x /= self.buffer_size
        mean_pose.orientation.y /= self.buffer_size
        mean_pose.orientation.z /= self.buffer_size
        mean_pose.orientation.w /= self.buffer_size

        # Find detection closest to mean
        min_dist = float('inf')
        closest_detection = None
        for detection in self.detection_buffers[marker_id]:
            dist = (
                (detection.pose.position.x - mean_pose.position.x) ** 2 +
                (detection.pose.position.y - mean_pose.position.y) ** 2 +
                (detection.pose.position.z - mean_pose.position.z) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_detection = detection

        return closest_detection

    def image_callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # reduce high-frequency noise
        cv_img = cv2.GaussianBlur(cv_img, (5, 5), 0)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        parameters =  cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5 #nmr qr is 10, size of workstation4 and 5 is 5
        
        corners, ids, _ = cv2.aruco.detectMarkers(cv_img, aruco_dict, parameters=parameters)
        print("Image received!")
        

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)

            for index in range(len(ids)):
                current_corner = corners[index]
                # marker_id = ids[index][0]
                marker_id = np.array([[ids[index][0]]])
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(current_corner, self.marker_length, self.camera_matrix, self.camera_distortion)
                R_mat, _ = cv2.Rodrigues(rvec)
                R_4by4 = np.eye(4)
                R_4by4[:3, :3] = R_mat
                quaternion = tft.quaternion_from_matrix(R_4by4)
                cv2.aruco.drawDetectedMarkers(cv_img, [current_corner], marker_id)
            
                print(f"Detected marker with ID: {marker_id}")
                print(f"trans & rot are: [{tvec[0][0][0]}, {tvec[0][0][1]}, {tvec[0][0][2]}, {quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")

                pose_msg = Pose()
                pose_msg.orientation.x = quaternion[0]
                pose_msg.orientation.y = quaternion[1]
                pose_msg.orientation.z = quaternion[2]
                pose_msg.orientation.w = quaternion[3]
                pose_msg.position.x = tvec[0][0][0]
                pose_msg.position.y = tvec[0][0][1]
                pose_msg.position.z = tvec[0][0][2]
                marker_id_int = int(marker_id)            
                aruco_msg = ArucoPose()
                aruco_msg.id = marker_id_int
                aruco_msg.pose = pose_msg
                if marker_id_int not in self.detection_buffers:
                    self.detection_buffers[marker_id_int] = []
                # print('aruco_msg', aruco_msg)
                # adding
                self.detection_buffers[marker_id_int].append(aruco_msg)
                if len(self.detection_buffers[marker_id_int]) > self.buffer_size:
                    self.detection_buffers[marker_id_int].pop(0)
                filtered_detection = self.filter_detections(marker_id_int)
                if filtered_detection is not None:
                    # Publish the filtered detection instead of the current one
                    topic_name = "/aruco_pose/id_" + str(filtered_detection.id)
                    if topic_name not in self.marker_pubs:
                        self.marker_pubs[topic_name] = rospy.Publisher(topic_name, ArucoPose, queue_size=10)
                    self.marker_pubs[topic_name].publish(filtered_detection)
                    quaternion = [filtered_detection.pose.orientation.x, filtered_detection.pose.orientation.y, filtered_detection.pose.orientation.z, filtered_detection.pose.orientation.w]
                    rot_matrix = tft.quaternion_matrix(quaternion)
                    rot_matrix_3x3 = rot_matrix[:3, :3]  # extract the 3x3 matrix

                    # Update tf for the filtered detection
                    rvec, _ = cv2.Rodrigues(rot_matrix_3x3)
                    rot_matrix = cv2.Rodrigues(rvec)[0]
                    transformation_matrix = np.eye(4)
                    transformation_matrix[:3, :3] = rot_matrix
                    quaternion = tft.quaternion_from_matrix(transformation_matrix)
                    self.broadcast_tf(filtered_detection.id, filtered_detection.pose.position, quaternion)
                # adding ends

                '''
                topic_name = "/aruco_pose/id_" + str (id_)
                if topic_name not in self.marker_pubs:
                    self.marker_pubs[topic_name] = rospy.Publisher(topic_name, ArucoPose, queue_size = 10)
                self.marker_pubs[topic_name].publish(aruco_msg)
                # self.marker_pub.publish(aruco_msg)
                self.broadcast_tf(ids, tvec, quaternion)
                '''

            # cv2.imshow("Detected ArUco", cv_img)
            # cv2.imwrite("/home/VLM_test/aruco_detection_output.jpg", cv_img)

            cv2.waitKey(1)
            
if __name__ == '__main__':
    rospy.init_node('aruco_detector_node', anonymous=True)
    detector = ArucoDetector()
    rospy.spin()

