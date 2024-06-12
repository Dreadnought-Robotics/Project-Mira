#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from custom_msgs.msg import telemetry

bridge = CvBridge()
surface_reading = 1000
depth_constant = 0.1


def enhance_underwater_image(img):
    # img = cv2.imread(img_path)
    # if img is None:
    #    print("Error: Image not found.")
    #    return

    # Convert image to RGB (OpenCV loads images in BGR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # White balancing using the Gray World assumption
    scale = img.mean(axis=(0, 1))
    img_balanced = img * (scale.mean() / scale)

    # Clip the values to [0, 255] and convert to uint8
    img_balanced = np.clip(img_balanced, 0, 255).astype(np.uint8)

    # Convert to LAB color space
    lab = cv2.cvtColor(img_balanced, cv2.COLOR_RGB2LAB)
    l, a, b = cv2.split(lab)

    # Apply CLAHE to L-channel
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))

    # Convert back to RGB color space
    enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2RGB)

    return enhanced_img


def callback(msg):
    frame = bridge.compressed_imgmsg_to_cv2(msg)
    image_enhanced = enhance_underwater_image(frame)
    frame_raw = bridge.cv2_to_compressed_imgmsg(image_enhanced)
    enhanced_publisher.publish(frame_raw)

    # Sleep for a short time to control the publishing rate


def pressure_callback(msg):
    ext_pressure = msg.external_pressure
    depth = -((ext_pressure - surface_reading) * depth_constant)
    f = Float64()
    f.data = depth
    # depth_pub.publish(depth)


if __name__ == "__main__":
    rospy.init_node("translator_node_f")
    enhanced_publisher = rospy.Publisher(
        "/camera_front/image_enhanced", CompressedImage, queue_size=10
    )
    # info_publisher = rospy.Publisher('/camera/info', Image, queue_size=10)
    rospy.Subscriber("camera_front/image_raw/compressed", CompressedImage, callback)

    # depth_pub = rospy.Publisher("/master/depth", Float64, queue_size=1)

    ext_pressure_sub = rospy.Subscriber(
        "/master/telemetry", telemetry, pressure_callback
    )
    rospy.spin()
