#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Vector3

# global lower_rgb
# global upper_rgb
bridge = CvBridge()
lower_rgb = (0, 0, 0)
upper_rgb = (30, 100, 255)


def CLAHE(image):
    if len(image.shape) == 3:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray_image = image
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_image = clahe.apply(gray_image)
    if len(image.shape) == 3:
        clahe_image = cv2.cvtColor(clahe_image, cv2.COLOR_GRAY2BGR)
    return clahe_image


def white_balance(image):
    lab_image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab_image)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    balanced_lab_image = cv2.merge((cl, a, b))
    balanced_image = cv2.cvtColor(balanced_lab_image, cv2.COLOR_LAB2BGR)
    return balanced_image


def Contrast_Up(image):
    contrasted_image = cv2.convertScaleAbs(image, alpha=0.5, beta=0)
    return contrasted_image


def Contrast_Down(image):
    contrasted_image = cv2.convertScaleAbs(image, alpha=0.2, beta=0)
    return contrasted_image


def Brightness_Up(image):
    brightened_image = cv2.convertScaleAbs(image, alpha=2, beta=150)
    return brightened_image


def Brightness_Down(image):
    darkened_image = cv2.convertScaleAbs(image, alpha=1.0, beta=10)
    return darkened_image


def Angle_Normalize(angle):
    if angle > 90:
        angle = angle - 180
    return angle


def Masking(image):
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, lower_rgb, upper_rgb)
    new_img = cv2.bitwise_and(image, image, mask=mask)
    new_img = cv2.GaussianBlur(new_img, (55, 55), 0)
    kernel = np.array([[-1, -1, -1], [-1, 99, -1], [-1, -1, -1]])
    new_img = cv2.filter2D(new_img, -1, kernel)
    new_img = cv2.bilateralFilter(new_img, 9, 1000, 1000)
    return new_img


def Make_Hough_Lines(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    average_angle_deg = None
    average_angle = None
    sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)
    gradient_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
    threshold = 180
    edges = np.uint8(gradient_magnitude > threshold) * 255
    edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
    edges = cv2.convertScaleAbs(edges)
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
    if lines is not None:
        angles = []
        for rho, theta in lines[:, 0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            angles.append(theta)
        average_angle = np.mean(angles)
        average_angle_deg = np.degrees(average_angle)
    if average_angle_deg:
        return image, average_angle_deg
    else:
        return image, None


def main_Callback(msg, pub):
    cx = 0
    cy = 0
    frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    height, width, _ = frame.shape
    # print(width)
    crop_width = int(width / 2)
    crop_height = int(height / 2)
    crop_x = width - crop_width
    crop_y = 0

    part_height = height // 3

    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lower_rgb = (0, 0, 0)
    upper_rgb = (255, 100, 30)
    lower_threshold = np.array(lower_rgb, dtype=np.uint8)
    upper_threshold = np.array(upper_rgb, dtype=np.uint8)
    mask = cv2.inRange(img_rgb, lower_threshold, upper_threshold)
    # cv2.imshow("f", mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) > 200:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

    part1 = frame[0:part_height, :]
    part2 = frame[part_height : 2 * part_height, :]
    # part3 = frame[2 * part_height :, :]

    n_part1, avg1 = Make_Hough_Lines(Masking(part1))
    n_part2, avg2 = Make_Hough_Lines(Masking(part2))
    # n_part3, avg3 = Make_Hough_Lines(Masking(part3))

    # cv2.imshow("Part1", part1)
    # cv2.imshow("Part2", part2)
    # cv2.imshow("Part3", part3)
    # cv2.imshow("orig Part1", n_part1)
    # cv2.imshow("orig Part2", n_part2)
    cv2.imshow("orig Part3", frame)
    cv2.waitKey(1)
    # print(avg1)
    # print(avg2)
    if avg1 and avg2:

        # print(avg1)
        # print(avg2)
        # print(avg3)
        # print("\n\n\n")
        avg1 = Angle_Normalize(avg1)
        avg2 = Angle_Normalize(avg2)
        # avg3 = Angle_Normalize(avg3)

        Angle_diff = abs(avg1 - avg2)
        print(Angle_diff)
        m = Vector3()
        m.x = cx
        m.y = cy
        m.z = Angle_diff
        # ------------------------------------------------
        pub.publish(m)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break


if __name__ == "__main__":
    try:
        rospy.init_node("angle_publisher", anonymous=True)
        pub = rospy.Publisher("/pipeline/errors", Vector3, queue_size=10)
        rospy.Subscriber(
            "/camera_down/image_enhanced", CompressedImage, main_Callback, pub
        )
        print("Ff")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
