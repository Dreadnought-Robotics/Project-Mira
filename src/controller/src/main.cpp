#include <ros/ros.h> 
#include <controller/control_utils.hpp> // Include header file for control utilities
#include <controller/subs_utils.hpp> // Include header file for subscriber utilities
#include <custom_msgs/commands.h> // Include custom messages header file
#include <geometry_msgs/PoseStamped.h> 
#include <mavros_msgs/OverrideRCIn.h> // Include header file for MAVROS messages
#include <mavros_msgs/Mavlink.h> // Include header file for MAVLink messages

// Threshold Definition for yaw in degrees
#define threshold 8 

/**
 * @Main function for the AUV controller node.
 * 
 * This function initializes the ROS node, sets up publishers and subscribers,
 * and enters a loop to control the AUV's yaw movement based on received data.
 * ROS Node: "auv_controller"
 * Publisher Topic: "/master/commands"
 * Subscriber Topic: "enter_topic_name"
 * 
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Returns 0 upon successful completion
 */
int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;

    // Publisher for PWM commands
    ros::Publisher pwm_publisher = nh.advertise<custom_msgs::commands>("/master/commands", 1);

    // Subscriber for AUV data
    Subscriber subs(nh);

    // Initialize PID controllers for yaw and forward motion
    Control yaw, forward;
    yaw.kp = 3.69; // Proportional gain for yaw control
    yaw.ki = 0; // Integral gain for yaw control
    yaw.kd = 0; // Derivative gain for yaw control

    // Initialize time variables
    ros::Time init_time = ros::Time::now();

    // Main loop to control AUV
    while (ros::ok()) {
        // Create PWM command message
        custom_msgs::commands cmd_pwm;

        // Get current time
        ros::Time time_now = ros::Time::now();

        // Calculate PID control for yaw
        float pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time).toSec());

        // Check if yaw error exceeds threshold
        if (sqrt(pow(subs.yaw_error, 2)) > threshold) {
            cmd_pwm.yaw = pid_yaw; // Apply PID control for yaw
        } else {
            cmd_pwm.yaw = yaw.hold(); // Hold current yaw position
        }

        // Publish PWM command message
        pwm_publisher.publish(cmd_pwm);

        // Process any incoming ROS messages
        ros::spinOnce();
    }

    return 0; 
}

