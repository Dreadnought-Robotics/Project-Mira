#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <std_msgs/Float32MultiArray.h>

/**
 * @file auv_controller.cpp
 * @brief ROS node for controlling an Autonomous Underwater Vehicle (AUV).
 *
 * This node handles the control of the AUV's forward, lateral, depth, and yaw movements using PID controllers.
 * It publishes control commands and subscribes to necessary topics to receive sensor data.
 */

// Control parameters and PWM Commands
custom_msgs::commands cmd_pwm;        /**< Command message to control the AUV */
Control forward, depth, yaw, lateral; /**< PID controllers for different movements */

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;

    // Publisher for sending control commands
    ros::Publisher pwm_publisher = nh.advertise<custom_msgs::commands>("/master/commands", 1);

    // Subscriber for receiving data from sensors
    Subscriber subs(nh);

    // Initialize control parameters
    // Yaw control parameters
    yaw.kp = 0.35;
    yaw.ki = 0.027;
    yaw.kd = 5.0;

    // Depth control parameters
    depth.kp = 1;
    depth.ki = 0.087;
    depth.kd = 14.1;

    // Forward control parameters
    forward.kp = 0.125;
    forward.ki = 0.0;
    forward.kd = 0.15;

    // Lateral control parameters
    lateral.kp = 0.125;
    lateral.ki = 0.0;
    lateral.kd = 0.15;

    // Initialize arm status
    bool arm = false;
    ros::Time init_time = ros::Time::now();
    cmd_pwm.arm = false;

    // Main control loop
    while (ros::ok())
    {
        if (subs.armed == true)
        {
            cmd_pwm.arm = subs.armed;
            cmd_pwm.mode = "STABILIZE";
            ros::Time time_now = ros::Time::now();

            if (cmd_pwm.arm == true)
            {
                // Compute PID control values
                float pid_forward = forward.pid_control(subs.forward_error, (time_now - init_time).toSec(), true);
                float pid_lateral = lateral.pid_control(subs.lateral_error, (time_now - init_time).toSec(), false);
                float pid_depth = depth.pid_control(subs.depth_error, (time_now - init_time).toSec(), false);
                float pid_yaw = 0;

                // Use yaw control only if locked
                if (subs.yaw_locked == true)
                {
                    pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time).toSec(), false);
                }
                else
                {
                    pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time).toSec(), true);
                }

                // Update command values
                cmd_pwm.forward = pid_forward;
                cmd_pwm.lateral = pid_lateral;
                cmd_pwm.thrust = pid_depth;
                cmd_pwm.yaw = pid_yaw;

                // Special case handling
                if (subs.depth_external > 1225)
                {
                    cmd_pwm.forward = 1500;
                    cmd_pwm.lateral = 1500;
                }
            }
            // Publish the command message
            pwm_publisher.publish(cmd_pwm);
        }
        else
        {
            // Reset errors if not armed
            forward.emptyError();
            yaw.emptyError();
            depth.emptyError();
            lateral.emptyError();
        }

        // Process callbacks
        ros::spinOnce();
    }

    return 0;
}
