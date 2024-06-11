#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('video_publisher', anonymous=True)
    rate = rospy.Rate(60)  # Adjust the rate as needed

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Error opening camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    pub = rospy.Publisher('/camera_down/image_raw/compressed', CompressedImage, queue_size=1)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # frame = cv2.imread("/home/shasankgunturu/Desktop/b90.png")
        # frame.resize(480,640)
        cv2.imshow("bruh", frame)
        cv2.waitKey(1)
        # if ret:
        compressed_img_msg = bridge.cv2_to_compressed_imgmsg(frame)
        pub.publish(compressed_img_msg)
        # else:
        #     rospy.logerr("Error reading frame from camera")

        rate.sleep()

    # cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
