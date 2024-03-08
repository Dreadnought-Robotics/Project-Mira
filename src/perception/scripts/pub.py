#!/usr/bin/env python3

'''
Node to publish a video stream from a camera.
'''

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def main():
    '''
    Main function to initialize the ROS node and publish the video stream.
    ROS Node: "video_publisher"
    Publisher Topic: "/video_stream1"
    '''
    rospy.init_node('video_publisher', anonymous=True)
    rate = rospy.Rate(30)  # Adjust the rate as needed

    # Initialize the camera
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        rospy.logerr("Error opening camera")
        return

    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Initialize ROS publisher and CvBridge
    pub = rospy.Publisher('/video_stream1', CompressedImage, queue_size=1)
    bridge = CvBridge()

    # Main loop to capture and publish frames
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Convert frame to compressed image message and publish
            compressed_img_msg = bridge.cv2_to_compressed_imgmsg(frame)
            pub.publish(compressed_img_msg)
        else:
            rospy.logerr("Error reading frame from camera")

        rate.sleep()

    # Release the camera
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

