#include <controller/joystick_utils.hpp> // Inlude joystick controller header file
#include <ros/ros.h> 
#include <custom_msgs/commands.h> // Include custom message header file

/**
 * @Main function for the joy controller node.
 * 
 * Function initializes the ROS node(joy_controller_node), creates a controller object,
 * and enters a loop to process incoming messages.
 * 
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Returns 0 upon successful completion
 */

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "joy_controller_node");
    ros::NodeHandle nh;

    // Create controller object
    controller obj = controller(nh);

    // While ROS is runnig loop
    while (ros::ok()) { 
        ros::spinOnce(); // Process any incoming ROS messages

        // obj.publish_data(); // Publish data
        // ROS_INFO("Publishing command message");
    }
    return 0;
}

