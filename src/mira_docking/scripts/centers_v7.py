#!/usr/bin/python3
import rospy
import torch
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from yolov7.models.experimental import attempt_load
from yolov7.utils.general import non_max_suppression, scale_coords, xyxy2xywh
from yolov7.utils.datasets import letterbox

class YoloV7RosNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov7_ros_node', anonymous=True)

        # Load YOLOv7 model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = attempt_load('~/Downloads/best.pt', map_location=self.device)
        self.model.to(self.device).eval()

        # Subscriber to the compressed image topic
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.callback)

    def callback(self, msg):
        # Convert the ROS image message to a NumPy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Pre-process the image for YOLOv7
        img0 = img.copy()
        img = letterbox(img, new_shape=(640, 640))[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.float()
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        with torch.no_grad():
            pred = self.model(img)[0]

        # Apply NMS (non-max suppression)
        pred = non_max_suppression(pred, 0.25, 0.45, classes=None, agnostic=False)

        # Process detections
        for i, det in enumerate(pred):
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                for *xyxy, conf, cls in det:
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / 640).view(-1).tolist()
                    print(f'Class: {cls}, Confidence: {conf}, Bbox: {xywh}')

if __name__ == '__main__':
    yolo_node = YoloV7RosNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
