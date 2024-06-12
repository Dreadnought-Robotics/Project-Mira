#!/usr/bin/python3

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()
# Global variable to store the current mouse position
current_x, current_y = -1, -1

#    cv2.namedWindow("Frame")
global frame


# Function to get the RGB, HSV, and LAB values of a pixel at (x, y)
def get_pixel_values(image, x, y):
    # OpenCV uses BGR format by default
    bgr = image[y, x]
    b, g, r = bgr
    rgb = (r, g, b)

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = hsv_image[y, x]

    lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lab = lab_image[y, x]

    return rgb, hsv, lab


# Mouse callback function to update the coordinates
def mouse_callback(event, x, y, flags, param):
    global current_x, current_y
    if event == cv2.EVENT_LBUTTONDOWN:  # Mouse left button click event
        current_x, current_y = x, y
        if 0 <= current_x < frame.shape[1] and 0 <= current_y < frame.shape[0]:
            # Get RGB, HSV, and LAB values of the pixel
            rgb_values, hsv_values, lab_values = get_pixel_values(
                frame, current_x, current_y
            )
            print(
                f"Pixel at ({current_x}, {current_y}): RGB={rgb_values}, HSV={hsv_values}, LAB={lab_values}"
            )


def img_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # Check if the coordinates are valid
    if 0 <= current_x < frame.shape[1] and 0 <= current_y < frame.shape[0]:
        # Get RGB values of the pixel
        rgb_values = get_pixel_values(frame, current_x, current_y)
        # print(f"RGB values at ({current_x}, {current_y}): {rgb_values}")

    # Display the resulting frame
    cv2.imshow("Frame", frame)
    cv2.setMouseCallback("Frame", mouse_callback)
    cv2.waitKey(1)
    key = cv2.waitKey(1)
    if key == ord("q"):
        pass
    if key == ord("s"):
        print("\n")


if __name__ == "__main__":
    rospy.init_node("picker")
    rospy.Subscriber("/camera_down/image_enhanced", CompressedImage, img_callback)
    rospy.spin()

# Initialize the webcam
# cap = cv2.VideoCapture(0)


# # When everything is done, release the capture and close windows
# cap.release()
# cv2.destroyAllWindows()
