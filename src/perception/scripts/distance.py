#!/usr/bin/python3

''' 
Script to measure distance between the camera and the center of Aruco markers and publish it
'''

import cv2 as cv
from cv2 import aruco
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Calibration data (for testing, replace with actual calibration data)
calib_data_path = "/home/shasankgunturu/catkin_ws/src/perception/scripts/data.npz"
cam_mat_mat = np.array([[928.665331, 0.000000, 367.020604], [0.000000, 924.329998, 231.120051], [0.0, 0.0, 1.0]])
dist_coef = np.array([0.558767, -0.383271, 0.005501, 0.038465, 0.000000])

# Marker size in centimeters
MARKER_SIZE = 14.5

# Initialize Aruco marker dictionary and parameters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# Initialize CvBridge
bridge = CvBridge()

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

if __name__ == "__main__":
    '''
    ROS Node: "detect_the_center"
    Publisher Topic: "/aruco/coordinates"
    Subscriber Topic: "/video_stream1"
    '''
    rospy.init_node('detect_the_center', anonymous=False)
    coordinate_publisher = rospy.Publisher("/aruco/coordinates", Vector3, queue_size=10)
    camera_sub = rospy.Subscriber("/video_stream1", CompressedImage, callbackk)
    rospy.spin()

