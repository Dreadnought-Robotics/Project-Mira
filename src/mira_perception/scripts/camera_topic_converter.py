#!/usr/bin/python3

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
bridge = CvBridge()

def callback(msg, image_publisher):
    frame = bridge.compressed_imgmsg_to_cv2(msg)
    frame_raw = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    image_publisher.publish(frame_raw)

        # Sleep for a short time to control the publishing rate

if __name__ == '__main__':
    rospy.init_node('my_camera_node')
    image_publisher = rospy.Publisher('/camera_down/image_raw2', Image, queue_size=10)
    # info_publisher = rospy.Publisher('/camera/info', Image, queue_size=10)
    rospy.Subscriber("camera_down/image_raw/compressed", CompressedImage, callback, image_publisher)
    rospy.spin()