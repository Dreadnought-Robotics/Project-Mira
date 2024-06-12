#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera_down/enhanced", CompressedImage, self.image_callback)
        self.image = None




    def get_pixel_rgb(self, x, y):
        if self.image is not None:
            try:
                # Get the RGB values of the pixel at (x, y)
                pixel_value = self.image[y, x]
                return pixel_value
            except IndexError:
                return None
        else:
            return None
def mouse_callback(event, x, y, flags, params):
    if event == 2:
        print(f"coords {x, y}, colors Blue- {img[y, x, 0]} , Green- {img[y, x, 1]}, Red- {img[y, x, 2]} ")
def image_callback(self, msg):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            print(e)
def main():
    image_subscriber = ImageSubscriber()

    while not rospy.is_shutdown():
        # Get mouse position
        x, y = cv2.getMousePos()
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", mouse_callback)
        # Get RGB values of the pixel under the mouse pointer
        pixel_rgb = image_subscriber.get_pixel_rgb(int(x), int(y))
        if pixel_rgb is not None:
            print("RGB values of pixel at ({},{}): {}".format(int(x), int(y), pixel_rgb))

        cv2.waitKey(1)

if __name__ == '__main__':
    main()