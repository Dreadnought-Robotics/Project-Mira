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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# cam_mat_mat = np.array([[482.709442, 0.000000, 312.481653], [0.000000, 484.938377, 251.272611], [0.0, 0.0, 1.0]])
cam_mat_mat = np.array([[2004.240793, 0.000000, 974.445467], [0.000000, 1972.754783, 591.922041], [0.000000, 0.000000, 1.000000]])


# dist_coef = calib_data["distortion_coefficients"]
# print(dist_coef)
# dist_coef = np.array([0.558767, -0.383271, 0.005501, 0.038465, 0.000000])
# dist_coef = np.array([-0.446214, 0.163514, -0.002405, -0.018243, 0.000000])
dist_coef = np.array([-0.459209, 0.259088, -0.000816, -0.000174, 0.000000])


calib_data_path = "/home/shasankgunturu/catkin_ws/src/perception/scripts/data.npz"

bridge = CvBridge()

# calib_data = np.load(calib_data_path, allow_pickle=True)

# cam_mat = calib_data["camera_matrix"]
# cam_mat_list = cam_mat.tolist()
# cam_mat_temp = [val for val in cam_mat_list["data"]]
# cam_mat_mat = np.array(cam_mat_temp).reshape(3, 3)
# cam_mat_mat = np.array([[482.709442, 0.000000, 312.481653], [0.000000, 484.938377, 251.272611], [0.0, 0.0, 1.0]])

# dist_coef = calib_data["distortion_coefficients"]
# print(dist_coef)
# dist_coef = np.array([0.558767, -0.383271, 0.005501, 0.038465, 0.000000])
# dist_coef = np.array([-0.446214, 0.163514, -0.002405, -0.018243, 0.000000])
# dist_coef = np.array([-0.411872, 0.155293, -0.005662, 0.001483, 0.000000])
# dist_coef = np.array([val for val in dist_coef_list])
# print(cam_mat_mat.shape, dist_coef.shape)

MARKER_SIZE = 14.5  # centimeters

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
print(marker_dict)
param_markers = aruco.DetectorParameters_create()
print(param_markers)

# Correct the interpolate_waypoints function if necessary to handle 3D points
def interpolate_waypoints(start, end, step):
    distance = np.linalg.norm(end - start)
    steps = int(distance / step)
    direction = (end - start) / distance
    return [start + direction * step * i for i in range(steps + 1)]




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
            # Assuming tvec represents the translation of the ArUco marker from the camera in world coordinates
            marker_point_3D = np.array([tVec[0][0]], dtype=np.float32).reshape(-1, 3)
            # Projected point on the XY plane (assuming Z=0 for the projection)
            projected_point_3D = np.array([(tVec[0][0][0], tVec[0][0][1], 0)], dtype=np.float32).reshape(-1, 3)

            # Assuming the origin is at (0,0,0) in 3D world coordinates
            origin_3D = np.array([0, 0, 0], dtype=np.float32)

            waypoints_origin_to_projected = interpolate_waypoints(origin_3D, projected_point_3D[0], 10)
            waypoints_projected_to_marker = interpolate_waypoints(projected_point_3D[0], marker_point_3D[0], 10 )
            waypoints_array = np.concatenate((waypoints_origin_to_projected, waypoints_projected_to_marker))

        
            '''------------------------------ PLOTTING -----------------------------'''
            
            
            '''# Follow with your plotting logic...
            fig = plt.figure(figsize=(12, 6))
            ax = fig.add_subplot(121, projection='3d')

            # Plot waypoints
            for point in waypoints_origin_to_projected:
                ax.scatter(*point, color='blue')
            for point in waypoints_projected_to_marker:
                ax.scatter(*point, color='red')

            # Plot lines for visual aid
            ax.plot(*zip(origin_3D, projected_point_3D[0]), color='blue', linestyle='dashed')
            ax.plot(*zip(projected_point_3D[0], marker_point_3D[0]), color='red', linestyle='dashed')

            # Annotate the marker point with its coordinates in green color
            marker_label = f"Aruco Marker ({marker_point_3D[0][0]:.2f}, {marker_point_3D[0][1]:.2f}, {marker_point_3D[0][2]:.2f})"
            ax.scatter(*marker_point_3D[0], color='green')  # Plot the marker point in green
            ax.text(marker_point_3D[0][0], marker_point_3D[0][1], marker_point_3D[0][2], marker_label, color='green')

            # Setting labels
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            ax_2d = fig.add_subplot(122)
            # Convert the last frame from BGR to RGB (matplotlib expects RGB format)
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            ax_2d.imshow(frame_rgb)
            ax_2d.axis('off')  # Hide axis
            ax_2d.set_title('Last Captured Frame')  '''


            '''-------------------------------- PLOTTING --------------------------------'''
            no_of_wp = len(waypoints_origin_to_projected) + len(waypoints_projected_to_marker)
            print(no_of_wp)

            waypoints_3d = [tVec[0][0] * (i / 4) for i in range(1, 4)]
            waypoints_image = [cv.projectPoints(np.array([wp]), rVec, tVec, cam_mat_mat, dist_coef)[0] for wp in waypoints_3d]
            # Display waypoints on the console
            for wp in waypoints_image:
                for point in wp:
                    point = tuple(point.ravel().astype(int))
                    cv.circle(frame, point, 5, (255, 0, 0), -1)
            
            print(f"Waypoint {i}: {marker_point_3D[-1]}")
            #waypoint_array_msg = []
            # for i, waypoint in enumerate(waypoints_array , 1):

                # print(f"Waypoint {i}: {waypoint}")
                # msg_wp = Vector3()
                # msg_wp.x = waypoint[1]
                # msg_wp.y = waypoint[0]
                # msg_wp.z = waypoint[2]
                # waypoint_array_msg.append(msg_wp)


                

            # for i, waypoint in enumerate(waypoints_3d, 1):
            #     print(f"Waypoint {i}: {waypoint}")
            #     cv.putText(frame, f"W{i}: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f})", (10, 30 * i), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
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
        coordinates = Vector3()
        coordinates.x = marker_point_3D[0][1]
        coordinates.y = marker_point_3D[0][0]
        coordinates.z = round(distance, 2)
        coordinate_publisher.publish(coordinates)
        print (coordinates)

        waypoint = Vector3()
        waypoint.x = waypoints_array[1][1]
        waypoint.y = waypoints_array[1][0]
        waypoint.z = waypoints_array[1][2]
        waypoints_publisher.publish(waypoint)



    else:
        pass
    cv.imshow("frame", frame)
    cv.waitKey(1)

        # print(sum_x)
        # print(sum_y)
        # print(z_coordinate/4)


if __name__ == "__main__":
    rospy.init_node('detect_the_center', anonymous=False)
    coordinate_publisher = rospy.Publisher("/aruco/coordinates", Vector3, queue_size=10)
    waypoints_publisher = rospy.Publisher("/aruco/wayypoints",Vector3,queue_size=10 )
    camera_sub = rospy.Subscriber("/camera_down/image_raw/compressed", CompressedImage, callbackk)
    rospy.spin()    