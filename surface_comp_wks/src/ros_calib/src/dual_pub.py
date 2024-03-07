#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('video_publisher', anonymous=True)
    rate = rospy.Rate(30)  # Adjust the rate as needed

    # Camera 1
    cap1 = cv2.VideoCapture(0)
    if not cap1.isOpened():
        rospy.logerr("Error opening camera 1")
        return

    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    pub1 = rospy.Publisher('/video_stream1', CompressedImage, queue_size=1)
    bridge1 = CvBridge()

    # Camera 2
    cap2 = cv2.VideoCapture(2)
    if not cap2.isOpened():
        rospy.logerr("Error opening camera 2")
        return

    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    pub2 = rospy.Publisher('/video_stream2', CompressedImage, queue_size=1)
    bridge2 = CvBridge()

    while not rospy.is_shutdown():
        # Camera 1
        ret1, frame1 = cap1.read()
        if ret1:
            compressed_img_msg1 = bridge1.cv2_to_compressed_imgmsg(frame1)
            pub1.publish(compressed_img_msg1)
        else:
            rospy.logerr("Error reading frame from camera 1")

        # Camera 2
        ret2, frame2 = cap2.read()
        if ret2:
            compressed_img_msg2 = bridge2.cv2_to_compressed_imgmsg(frame2)
            pub2.publish(compressed_img_msg2)
        else:
            rospy.logerr("Error reading frame from camera 2")

        rate.sleep()

    cap1.release()
    cap2.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
