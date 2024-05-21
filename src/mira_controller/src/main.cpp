#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <std_msgs/Char.h>
#include <std_msgs/Float32MultiArray.h>

/**
 * @brief Main control node for the AUV (Autonomous Underwater Vehicle).
 * 
 * This node handles the control of the AUV using PID controllers for forward,
 * depth, yaw, and lateral movements. It subscribes to sensor data and publishes
 * command signals to control the AUV.
 */

// Bot Orientation
// +ve x  -ve (CCW)
// (CW) |
// y -- bot
// +ve

bool forward_bool = false;
#define THRESHOLD 8 // degrees

custom_msgs::commands cmd_pwm;
Control forward, depth, yaw, lateral;

int main(int argc, char** argv) {
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;

    ros::Publisher pwm_publisher = nh.advertise<custom_msgs::commands>("/master/commands", 1);

    // Initialize PID controller parameters
    yaw.kp = 0.35;
    yaw.ki = 0.027;
    yaw.kd = 5.0;

    depth.kp = 0.93;
    depth.ki = 0.087;
    depth.kd = 12.1;

    forward.kp = 0.125;
    forward.ki = 0.0;
    forward.kd = 0.15;

    lateral.kp = 0.125;
    lateral.ki = 0.0;
    lateral.kd = 0.15;

    ros::Time init_time = ros::Time::now();
    cmd_pwm.arm = false;

    while (ros::ok()) {
        if (subs.autonomy_switch && subs.rov_commands.arm) {
            cmd_pwm.arm = subs.rov_commands.arm;
            cmd_pwm.mode = "STABILIZE";

            ros::Time time_now = ros::Time::now();

            if (cmd_pwm.arm) {
                float dt = (time_now - init_time).toSec();
                float pid_forward = forward.pid_control(subs.forward_error, dt, true);
                float pid_lateral = lateral.pid_control(subs.lateral_error, dt, false);
                float pid_depth = depth.pid_control(subs.depth_error, dt, false);
                float pid_yaw = subs.yaw_locked ? yaw.pid_control(subs.yaw_error, dt, false) : yaw.pid_control(subs.yaw_error, dt, true);

                cmd_pwm.forward = pid_forward;
                cmd_pwm.lateral = pid_lateral;
                cmd_pwm.thrust = pid_depth;
                cmd_pwm.yaw = pid_yaw;

                if (subs.depth_external > 1100) {
                    cmd_pwm.forward = 1500;
                    cmd_pwm.lateral = 1500;
                }
            }

            pwm_publisher.publish(cmd_pwm);
        } else {
            forward.emptyError();
            yaw.emptyError();
            depth.emptyError();
            lateral.emptyError();
        }

        ros::spinOnce();
    }

    return 0;
}
