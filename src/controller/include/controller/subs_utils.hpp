#include <ros/ros.h> 
#include <geometry_msgs/Vector3.h> // Include Vector3 from geometery_msgs

/**
 * @brief Subscriber class for receiving telemetry data.
 */
class Subscriber {
public:
    // Public member variables for storing telemetry data
    double thrust_error, yaw_error, forward_error;
    double angular_velocity_z;

    /**
     * @brief Constructor for Subscriber class.
     * 
     * @param nh NodeHandle object for ROS node
     */
    Subscriber(ros::NodeHandle nh) {
        // Subscribe to telemetry topic "/aruco/coordinates" with a queue size of 1
        telemetry_sub = nh.subscribe("/aruco/coordinates", 1, &Subscriber::telemetryCallback, this);
    }

private:
    ros::Subscriber telemetry_sub; // ROS subscriber for telemetry data

    /**
     * @brief Callback function for telemetry messages.
     * 
     * @param msg Pointer to the received telemetry message
     */
    void telemetryCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        // Calculate yaw error from telemetry data
        yaw_error = std::atan2(msg->y, msg->x) * 57.2958; // Convert radians to degrees
    }
};

