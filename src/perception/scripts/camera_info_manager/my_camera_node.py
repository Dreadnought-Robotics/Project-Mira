#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import cv2
from camera_info_manager import CameraInfoManager

# Create a camera info manager to handle camera_info
camera_info_manager = CameraInfoManager(cname='my_camera_node', url='file:///shasankgunturu/catkin_ws/src/perception/scripts/my_camera_node.yaml', namespace='camera/')
camera_info_manager.loadCameraInfo()

# Create a publisher for the camera info
bridge = CvBridge()


def callback(msg):    
        
        frame = bridge.compressed_imgmsg_to_cv2(msg)
        
        # Convert the OpenCV image to a ROS image message
        img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # Publish the ROS image message
        image_publisher.publish(img_msg)
                # Publish camera info
        camera_info = camera_info_manager.getCameraInfo()
        camera_info.header.stamp = rospy.Time.now()
        camera_info_publisher.publish(camera_info)


        # Sleep for a short time to control the publishing rate

if __name__ == '__main__':
    rospy.init_node('my_camera_node')
    image_publisher = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    camera_info_publisher = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    camera_sub = rospy.Subscriber('/video_stream1', CompressedImage, callback)
    rospy.spin()

