#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

/**
 * @brief Callback function to handle compressed image messages.
 * 
 * @param msg The received compressed image message.
 */
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        // Convert compressed image message to OpenCV image format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        // Display or process the decompressed image as needed
        cv::imshow("Decompressed Image", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        // Handle cv_bridge exceptions
        ROS_ERROR("CV_Bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "video_subscriber2");
    ros::NodeHandle nh;

    // Subscribe to the topic publishing compressed images
    ros::Subscriber sub = nh.subscribe("/camera_front/image_raw/compressed", 1, imageCallback);

    // Spin until ROS is shutdown
    ros::spin();

    return 0;
}

