#!/usr/bin/python3

''' 
Script to measure distance between the camera and the center of Aruco markers and publish it
'''

import cv2 as cv
from cv2 import aruco
import numpy as np
import rospy, sys
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge


# Marker size in centimeters
MARKER_SIZE = 14.5

# Initialize Aruco marker dictionary and parameters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
param_markers = aruco.DetectorParameters_create()

# Initialize CvBridge
bridge = CvBridge()
cam_mat_mat = np.array([[928.665331, 0.000000, 367.020604], [0.000000, 924.329998, 231.120051], [0.0, 0.0, 1.0]])
dist_coef = np.array([0.558767, -0.383271, 0.005501, 0.038465, 0.000000])

def callback_camera_info(data):
    cam_mat_mat = np.array(data.K).reshape(3,3)
    dist_coef = np.array(data.D)

def callbackk(msg):
    ''' 
    Callback function to process camera images, detect Aruco markers, estimate their pose, 
    and publish the coordinates of marker centers.

    Args:
        msg (sensor_msgs.msg.CompressedImage): Compressed image message from the camera.

    '''
    # Convert ROS CompressedImage message to OpenCV image format
    frame = bridge.compressed_imgmsg_to_cv2(msg)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
    if marker_IDs:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            marker_center = np.mean(corners, axis=0)
            marker_center = tuple(marker_center.astype(int))
            distance = np.sqrt(
                tVec[i][0][2] * 2 + tVec[i][0][0] * 2 + tVec[i][0][1] ** 2
            )
            waypoints_3d = [tVec[0][0]]
            print(waypoints_3d[0][2])
            cv.putText(frame, waypoints_3d[0][2], cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    # Detect Aruco markers in the frame
    marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_IDs:
        # Estimate pose of detected Aruco markers
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat_mat, dist_coef)

        # Publish coordinates of the marker center
        publish_coordinates = tVec[0][0]
        coordinates = Vector3()
        coordinates.x = -publish_coordinates[1]
        coordinates.y = -publish_coordinates[0]
        coordinates.z = round(publish_coordinates[2], 2)
        coordinate_publisher.publish(coordinates)
        
    cv.imshow("Aruco_Distance", frame)
    cv.waitKey(1)

if __name__ == "_main_":
    '''
    ROS Node: "detect_the_center"
    Publisher Topic: "/aruco/coordinates"
    Subscriber Topic: give argument
    '''
    publish_topic = sys.argv[1]
    rospy.init_node('detect_the_center', anonymous=False)
    coordinate_publisher = rospy.Publisher("/aruco/coordinates", Vector3, queue_size=10)
    camera_sub = rospy.Subscriber(publish_topic, CompressedImage, callbackk)
    camera_info = rospy.Subscriber("/camera_down/camera_info", CameraInfo, callback_camera_info)
    rospy.spin()