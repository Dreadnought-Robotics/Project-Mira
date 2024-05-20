#!/usr/bin/python3
import rospy
import torch
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from yolov7.models.experimental import attempt_load
from yolov7.utils.general import non_max_suppression, scale_coords, xyxy2xywh
from yolov7.utils.datasets import letterbox
from geometry_msgs.msg import Vector3

class YoloV7RosNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov7_ros_node', anonymous=True)

        # Load YOLOv7 model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = attempt_load('/home/f3rt/Project-Mira/src/mira_docking/scripts/best.pt', map_location=self.device)
        self.model.to(self.device).eval()
        self.centre_pub = rospy.Publisher('/docking/center', Vector3, queue_size=1)
        # Subscriber to the compressed image topic
        self.image_sub = rospy.Subscriber('/camera_down/image_raw/compressed', CompressedImage, self.callback)

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

        center_msg = Vector3()

        # Process detections
        for i, det in enumerate(pred):
            if len(det):
                
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
                x_min = 99999
                y_min = 99999
                prev_msg_x = x_min
                prev_msg_y = y_min
                # Draw bounding boxes
                for *xyxy, conf, cls in det:
                    if conf > 0.8:
                        xyxy = [int(x) for x in xyxy]  # Convert to integers
                        x_centre = (xyxy[0] + xyxy[2])//2
                        y_centre = (xyxy[1] + xyxy[3])//2
                        if y_min > y_centre:
                            y_min = y_centre
                            x_min = x_centre
                        if (x_min>640 or y_min>600):
                            center_msg.x = prev_msg_x
                            center_msg.y = prev_msg_y
                        else:
                            center_msg.x = x_min
                            center_msg.y = y_min
                        self.centre_pub.publish(center_msg)
                        # cv2.rectangle(img0, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)  # Draw rectangle
                                #label = f'Class: {cls}, Confidence: {conf:.2f}'
                                #cv2.putText(img0, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # Put label
                        cv2.circle(img0, (x_min,y_min), 3, (255,0,0), -1)
                        prev_msg_x = x_min
                        prev_msg_y = y_min
        # Display annotated image
        cv2.imshow("YOLOv7 Object Detection", img0)
        cv2.waitKey(1)

if __name__ == '__main__':
    yolo_node = YoloV7RosNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #cv2.destroyAllWindows()
