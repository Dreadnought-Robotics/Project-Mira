#!/usr/bin/python3

import os

os.system(
    "export PYTHONPATH=$PYTHONPATH:/home/f3rt/Project-Mira/src/mira_docking/scripts/yolov7/"
)

import cv2
import numpy as np
import rospkg
import rospy
import torch
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CompressedImage
from yolov7.models.experimental import attempt_load
from yolov7.utils.datasets import letterbox
from yolov7.utils.general import non_max_suppression, scale_coords


class YoloV7RosNode:
    """
    YOLOv7 ROS node for object detection.
    """

    def __init__(self):
        """
        Initializes the YOLOv7 ROS node.
        """
        rospy.init_node("yolov7_ros_node", anonymous=True)

        # Load YOLOv7 model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("mira_docking")

        # Load YOLOv7 model
        model_path = os.path.join(package_path, "scripts/best.pt")
        self.model = attempt_load(model_path, map_location=self.device)

        self.model.to(self.device).eval()
        self.center_pub = rospy.Publisher("/docking/center", Vector3, queue_size=1)

        # Subscriber to the compressed image topic
        self.image_sub = rospy.Subscriber(
            "/camera_down/image_raw/compressed", CompressedImage, self.callback
        )

    def callback(self, msg):
        """
        Callback function for processing incoming image messages.
        """
        np_image = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_image, cv2.IMREAD_COLOR)

        # Pre-process the image for YOLOv7
        image_original = image.copy()
        image = letterbox(image, new_shape=(640, 640))[0]
        image = image[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        image = np.ascontiguousarray(image)
        image_tensor = torch.from_numpy(image).to(self.device)
        image_tensor = image_tensor.float()
        image_tensor /= 255.0  # 0 - 255 to 0.0 - 1.0
        if image_tensor.ndimension() == 3:
            image_tensor = image_tensor.unsqueeze(0)

        # Inference
        with torch.no_grad():
            predictions = self.model(image_tensor)[0]

        # Apply NMS (non-max suppression)
        predictions = non_max_suppression(
            predictions, 0.25, 0.45, classes=None, agnostic=False
        )

        center_msg = Vector3()

        # Process detections
        for i, detection in enumerate(predictions):
            if len(detection):
                detection[:, :4] = scale_coords(
                    image_tensor.shape[2:], detection[:, :4], image_original.shape
                ).round()
                min_x = 99999
                min_y = 99999
                prev_x = min_x
                prev_y = min_y

                # Draw bounding boxes
                for *xyxy, confidence, cls in detection:
                    if confidence > 0.8:
                        xyxy = [int(x) for x in xyxy]  # Convert to integers
                        center_x = (xyxy[0] + xyxy[2]) // 2
                        center_y = (xyxy[1] + xyxy[3]) // 2
                        if min_y > center_y:
                            min_y = center_y
                            min_x = center_x
                        if min_x > 640 or min_y > 600:
                            center_msg.x = prev_x
                            center_msg.y = prev_y
                        else:
                            center_msg.x = min_x
                            center_msg.y = min_y
                        self.center_pub.publish(center_msg)
                        # cv2.circle(image_original, (min_x, min_y), 3, (255, 0, 0), -1)
                        prev_x = min_x
                        prev_y = min_y
        # Display annotated image
        # cv2.imshow("YOLOv7 Object Detection", image_original)
        # cv2.waitKey(1)


if __name__ == "__main__":
    yolo_node = YoloV7RosNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
