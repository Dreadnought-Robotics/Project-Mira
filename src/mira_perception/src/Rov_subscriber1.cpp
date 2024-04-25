#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::flip(cv_ptr->image, cv_ptr->image, 0);
        cv::resizeWindow("Camera Front HD View", 1920, 1080);
        cv::moveWindow("Camera Front HD View", 960, 540);
        cv::imshow("Camera Front Hd View", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV_Bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rov_feed_front");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camer_front_hd/image_raw/compressed", 1, imageCallback);

    ros::spin();

    return 0;
}

