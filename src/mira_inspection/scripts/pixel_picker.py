#!/usr/bin/python3

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()
# Global variable to store the current mouse position
current_x, current_y = -1, -1

#    cv2.namedWindow("Frame")


# Function to get the RGB values of a pixel at (x, y)
def get_pixel_rgb(image, x, y):
    # OpenCV uses BGR format by default
    b, g, r = image[y, x]
    return (r, g, b)


# Mouse callback function to update the coordinates
def mouse_callback(event, x, y, flags, param):
    global current_x, current_y
    if event == cv2.EVENT_MOUSEMOVE:  # Mouse move event
        current_x, current_y = x, y


def img_callback(msg):
    frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # Check if the coordinates are valid
    if 0 <= current_x < frame.shape[1] and 0 <= current_y < frame.shape[0]:
        # Get RGB values of the pixel
        rgb_values = get_pixel_rgb(frame, current_x, current_y)
        print(f"RGB values at ({current_x}, {current_y}): {rgb_values}")

    # Display the resulting frame
    cv2.imshow("Frame", frame)
    cv2.setMouseCallback("Frame", mouse_callback)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("picker")
    rospy.Subscriber("/camera_down/image_enhanced", CompressedImage, img_callback)
    rospy.spin()

# Initialize the webcam
# cap = cv2.VideoCapture(0)


# # When everything is done, release the capture and close windows
# cap.release()
# cv2.destroyAllWindows()
