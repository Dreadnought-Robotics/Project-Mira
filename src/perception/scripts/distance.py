#!/usr/bin/python3
#############################################################################################
#script to measure distance b/w the camera and the center of 4 aruco markers and publish it
#############################################################################################
import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
calib_data_path = "/home/shasankgunturu/catkin_ws/src/perception/scripts/data.npz"

bridge = CvBridge()
import numpy as np

# calib_data = np.load(calib_data_path, allow_pickle=True)

# cam_mat = calib_data["camera_matrix"]
# cam_mat_list = cam_mat.tolist()
# cam_mat_temp = [val for val in cam_mat_list["data"]]
# cam_mat_mat = np.array(cam_mat_temp).reshape(3, 3)
cam_mat_mat = np.array([[928.665331, 0.000000, 367.020604], [0.000000, 924.329998, 231.120051], [0.0, 0.0, 1.0]])
# dist_coef = calib_data["distortion_coefficients"]
# print(dist_coef)
dist_coef = np.array([0.558767, -0.383271, 0.005501, 0.038465, 0.000000])
# dist_coef = np.array([val for val in dist_coef_list])

# print(cam_mat_mat.shape, dist_coef.shape)

MARKER_SIZE = 14.5  # centimeters

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
print(marker_dict)
param_markers = aruco.DetectorParameters_create()
print(param_markers)


def callbackk(msg):
    frame = bridge.compressed_imgmsg_to_cv2(msg)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
    if marker_IDs:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            marker_center = np.mean(corners, axis=0)
            marker_center = tuple(marker_center.astype(int))
            distance = np.sqrt(
                tVec[i][0][2] * 2 + tVec[i][0][0] * 2 + tVec[i][0][1] ** 2
            )
            waypoints_3d = [tVec[0][0] * (i / 4) for i in range(1, 4)]
            waypoints_3d.append(tVec[0][0])
            # waypoints_image = [cv.projectPoints(np.array([wp]), rVec, tVec, cam_mat_mat, dist_coef)[0] for wp in waypoints_3d]
            # Display waypoints on the console
            # for wp in waypoints_image:
            #     for point in wp:
            #         point = tuple(point.ravel().astype(int))
            #         cv.circle(frame, point, 5, (255, 0, 0), -1)
            for i, waypoint in enumerate(waypoints_3d, 1):
                # print(f"Waypoint {i}: {waypoint}")
                    # waypoint[0] = -waypoint[0]
                    # waypoint[1] = -waypoint[1]
                    cv.putText(frame, f"W{i}: ({-waypoint[1]:.2f}, {-waypoint[0]:.2f}, {waypoint[2]:.2f})", (10, 30 * i), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # Draw the pose of the marker
            # point = cv.drawFrameAxes(frame, cam_mat_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            # cv.putText(
            #     frame,
            #     f"id: {ids[0]} Dist: {round(distance, 2)} Coords: {marker_center[0], marker_center[1]}",
            #     top_right,
            #     cv.FONT_HERSHEY_PLAIN,
            #     1.3,
            #     (0, 0, 255),
            #     2,
            #     cv.LINE_AA,
            # )
        publish_coordinates = waypoints_3d[-1]
        print(publish_coordinates)
        coordinates = Vector3()
        coordinates.x = -publish_coordinates[1]
        coordinates.y = -publish_coordinates[0]
        coordinates.z = round(publish_coordinates[2], 2)
        coordinate_publisher.publish(coordinates)
    else:
        pass
    cv.imshow("bleh", frame)
    cv.waitKey(1)

        # print(sum_x)
        # print(sum_y)
        # print(z_coordinate/4)


if __name__ == "__main__":
    rospy.init_node('detect_the_center', anonymous=False)
    coordinate_publisher = rospy.Publisher("/aruco/coordinates", Vector3, queue_size=10)
    camera_sub = rospy.Subscriber("/video_stream1", CompressedImage, callbackk)
    rospy.spin()