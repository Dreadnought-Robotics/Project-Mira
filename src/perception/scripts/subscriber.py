#!/usr/bin/python3
'''
Script to detect Aruco markers in a video stream and estimate their pose.
'''

import cv2 as cv
from cv2 import aruco
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Path to calibration data
calib_data_path = "/home/shasankgunturu/calibration_ws/src/calib_pkg/scripts/calib_data/MultiMatrix.npz"

# Load calibration data
calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

# Aruco marker size in centimeters
MARKER_SIZE = 8

# Initialize Aruco marker dictionary and parameters
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# Initialize CvBridge
bridge = CvBridge()

def image_callback(msg):
    '''
    Callback function to process camera images, detect Aruco markers, estimate their pose, and display information.

    Args:
        msg (sensor_msgs.msg.CompressedImage): Compressed image message from the camera.
    '''
    try:
        # Convert ROS CompressedImage message to OpenCV image
        frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Detect Aruco markers in the frame
        marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

        if marker_corners:
            # Estimate pose of detected Aruco markers
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)

            # Display information on the frame
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                distance = np.linalg.norm(tVec[i][0])  # Euclidean distance
                cv.putText(frame, f"id: {ids[0]} Dist: {round(distance, 2)}", tuple(corners[0][0].astype(int)), cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv.LINE_AA)

        # Display the frame
        cv.imshow("frame", frame)
        cv.waitKey(1)
    except Exception as e:
        print(e)

if __name__ == '__main__':
    '''
    Main function to initialize the ROS node and subscribe to the video stream topic.
    ROS Node: "aruco_marker_detector"
    Subscriber Topic: "video_stream1"
    '''
    rospy.init_node('aruco_marker_detector', anonymous=True)
    rospy.Subscriber("video_stream1", CompressedImage, image_callback)
    rospy.spin()

