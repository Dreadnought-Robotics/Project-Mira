#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

cv::VideoCapture cap1(2);
cv::VideoCapture cap2(0);


void launch_cameras() {
    if (!cap1.isOpened()) {
        ROS_ERROR("Error opening camera 0.");
    }
    if (!cap2.isOpened()) {
        ROS_ERROR("Error opening camera 1.");
    }
    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;

    launch_cameras();

    // image_transport::ImageTransport it(nh);
    // image_transport::Publisher pub1 = it.advertise("video_stream1", 1);
    // image_transport::Publisher pub2 = it.advertise("video_stream2", 1);

    ros::Publisher pub1 = nh.advertise<sensor_msgs::CompressedImage>("video_stream1", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::CompressedImage>("video_stream2", 1);

    while (ros::ok()) {
        cv::Mat frame1;
        cv::Mat frame2;
        cap1 >> frame1;
        cap2 >> frame2;
            ROS_INFO("Frame 1 dimensions: %dx%d", frame1.cols, frame1.rows);
            ROS_INFO("Frame 2 dimensions: %dx%d", frame2.cols, frame2.rows);
            if (frame1.empty()) {
        ROS_ERROR("Empty frame received from camera 0.");
        continue; // Skip this iteration
            }
        if (frame2.empty()) {
        ROS_ERROR("Empty frame received from camera 1.");
        continue; // Skip this iteration
            }
        sensor_msgs::CompressedImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toCompressedImageMsg();
        sensor_msgs::CompressedImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toCompressedImageMsg();
        pub1.publish(msg1);
        pub2.publish(msg2);
    }
    cap1.release();
    cap2.release();
    return 0;
}
