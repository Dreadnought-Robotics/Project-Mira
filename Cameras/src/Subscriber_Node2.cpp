#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("Video Stream 2", image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV_Bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_subscriber2");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("video_stream2", 1, imageCallback);

    ros::spin();

    return 0;
}
