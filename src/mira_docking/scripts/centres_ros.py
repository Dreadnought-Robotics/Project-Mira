import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import time

class ImageProcessor:
    def _init_(self):
        rospy.init_node('image_processor_node', anonymous=True)
        self.bridge = CvBridge()
        self.video_path = '/home/tonyox/AUV/docking/12.webm'

        self.image_subscriber = rospy.Subscriber('/camera_down/image_raw/compressed', CompressedImage, self.image_callback)
        self.image_publisher = rospy.Publisher('cropped_image', CompressedImage, queue_size=1)

        self.prev_frame_time = 0
        self.fps = 0

    def image_callback(self, compressed_image_msg):
        np_arr = np.fromstring(compressed_image_msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        # Convert the received image to OpenCV format
        cv_image = self.bridge.compressed_imgmsg_to_cv2(compressed_image_msg, desired_encoding="bgr8")

        width = 640
        height = 480
        crop_width = int(width / 2)
        crop_height = int(height / 2)
        crop_x = width - crop_width
        crop_y = 0
        new_frame = cv_image[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]

        # Color thresholding
        lower_rgb = (50,110,90)
        upper_rgb =  (90,160,120)
        img_rgb = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)

        lower_threshold = np.array(lower_rgb, dtype=np.uint8)
        upper_threshold = np.array(upper_rgb, dtype=np.uint8)

        mask = cv.inRange(img_rgb, lower_threshold, upper_threshold)

        # Find contours in the binary mask
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        contour_frame = cv_image.copy()
        contour_frame2 = mask.copy()
        

        for cnt in contours:
            if(cv.contourArea(cnt)>200):
                M = cv.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    print(cx,cy)
                else:
                    cx, cy = 0, 0

                # Draw contours on the original frame
                contour_frame = cv.drawContours(cv_image.copy(), contours, -1, (0, 255, 0), 2)
                cv.circle(contour_frame, (cx,cy), 7, (0,0,255), -1)
                new_frame_time = time.time()

                fps = 1/(new_frame_time-self.prev_frame_time)
                self.prev_frame_time = new_frame_time

                fps = int(fps/100)
                fps = str(fps)

                contour_frame2 = cv.drawContours(mask.copy(), contours, -1, (0, 255, 0), 2)
                cv.circle(contour_frame2, (cx,cy), 7, (0,0,0), -1)

        # Display images
        cv.imshow("Contours", contour_frame)
        cv.imshow("Mask", contour_frame2)

        # Publish the processed image
        cropped_image_msg = self.bridge.cv2_to_compressed_imgmsg(new_frame)
        self.image_publisher.publish(cropped_image_msg)

        cv.waitKey(1)

if __name__ == '_main_':
    processor = ImageProcessor()
    rospy.spin()