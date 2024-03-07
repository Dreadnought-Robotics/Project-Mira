#!/usr/bin/python3
import cv2 as cv
from cv2 import aruco 
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

calib_data_path = "/home/shasankgunturu/calibration_ws/src/calib_pkg/scripts/calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 8  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()

bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert ROS CompressedImage message to OpenCV image
        frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                # Draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                # cv.putText(
                #     frame,
                #     #what happening here 
                #     #f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                #     bottom_right,
                #     cv.FONT_HERSHEY_PLAIN,
                #     1.0,
                #     (0, 0, 255),
                #     2,
                #     cv.LINE_AA,
                # )
                # print(ids, "  ", corners)
        cv.imshow("frame", frame)
        cv.waitKey(1)
    except Exception as e:
        print(e)

def main():
    rospy.init_node('aruco_marker_detector', anonymous=True)
    rospy.Subscriber("video_stream1", CompressedImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
