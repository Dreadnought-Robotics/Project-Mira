#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;

    cv::VideoCapture cap1(4);
    if (!cap1.isOpened()) {
        ROS_ERROR("Error opening camera 0.");
        return -1;
    }



    cv::VideoCapture cap2(2);
    if (!cap2.isOpened()) {
        ROS_ERROR("Error opening camera 1.");
        return -1;
    }

    ros::Publisher pub1 = nh.advertise<sensor_msgs::Image>("video_stream1", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::Image>("video_stream2", 1);

    ros::Rate loop_rate(40);


    while (ros::ok()) {
       
        cv::Mat frame1;
        cv::Mat frame2;

        cap1 >> frame1;
        cap2 >> frame2;

        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toImageMsg();

        pub1.publish(msg1);
        pub2.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
