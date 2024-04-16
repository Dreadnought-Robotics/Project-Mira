#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        // Display or process the decompressed image as needed
        cv::imshow("Decompressed Image", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV_Bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_subscriber1");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/video_stream1", 1, imageCallback);

    ros::spin();

    return 0;
}