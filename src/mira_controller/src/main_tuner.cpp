#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/tuning_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <custom_msgs/commands.h>
#include <std_msgs/Char.h>

// Coordinate System Description:
//   +ve   x  -ve (CCW)
//   (CW)  |
//   y -- bot
//   +ve

// PID controller instances for yaw, forward, and lateral control
Control yaw, forward, lateral;

// Initial PID constants
std::vector<double> pid_constants{0.35, 0.027, 5.1};

// Threshold for error adjustment
#define threshold 8 // degrees

// Command message for PWM control
custom_msgs::commands cmd_pwm;

/**
 * Callback function to handle key inputs.
 * Adjusts PID constants based on the received key.
 */
void keys_callback(const std_msgs::Char::ConstPtr &msg)
{
    char key = msg->data;
    if (key == 'q')
    {
        cmd_pwm.arm = false;
        ROS_INFO("Unarmed");
        forward.emptyError();
        yaw.emptyError();
        lateral.emptyError();
    }
    else if (key == 'p')
    {
        cmd_pwm.arm = true;
        ROS_INFO("Armed");
    }
    else if (key == 'w')
    {
        yaw.kp += 0.5;
        ROS_INFO("Current kp value: %f", yaw.kp);
    }
    else if (key == 's')
    {
        yaw.kp -= 0.5;
        ROS_INFO("Current kp value: %f", yaw.kp);
    }
    else if (key == 'e')
    {
        yaw.ki += 0.1;
        ROS_INFO("Current ki value: %f", yaw.ki);
    }
    else if (key == 'd')
    {
        yaw.ki -= 0.1;
        ROS_INFO("Current ki value: %f", yaw.ki);
    }
    else if (key == 'r')
    {
        yaw.kd += 0.1;
        ROS_INFO("Current kd value: %f", yaw.kd);
    }
    else if (key == 'f')
    {
        yaw.kd -= 0.1;
        ROS_INFO("Current kd value: %f", yaw.kd);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;

    ros::Publisher pwm_publisher = nh.advertise<custom_msgs::commands>("/master/commands", 1);
    ros::Publisher error_publisher = nh.advertise<custom_msgs::commands>("/yaw/error", 1);
    ros::Publisher cost_publisher = nh.advertise<std_msgs::Float32>("/cost", 1);
    ros::Subscriber keys_subscriber = nh.subscribe("keys", 1, keys_callback);

    Subscriber subs(nh);
    std::vector<double> learning_rate{0.1, 0.005, 1};
    GradientDescent descent(learning_rate);

    yaw.kp = pid_constants[0];
    yaw.ki = pid_constants[1];
    yaw.kd = pid_constants[2];

    bool arm = true;
    double init_time = ros::Time::now().toSec();
    double prev_time = ros::Time::now().toSec();
    cmd_pwm.arm = false;
    cmd_pwm.mode = "STABILIZE";

    float _h = 0.000001;
    double time_interval = 30.0;
    std::vector<double> cost_h{0, 0, 0};
    double cost;
    float dt = 0.05;
    ros::Rate loop_rate(1.0 / dt);

    std::vector<double> cost_values;
    int iterations = 0;
    bool increasing_h = false, increasing_h_kp = false, increasing_h_ki = false, increasing_h_kd = false;

    while (ros::ok())
    {
        double time_now = ros::Time::now().toSec();
        float pid_yaw;

        if (!cmd_pwm.arm)
        {
            prev_time = ros::Time::now().toSec();
            forward.emptyError();
            yaw.emptyError();
            lateral.emptyError();
        }
        else
        {
            if (increasing_h)
            {
                if (increasing_h_kp && (time_now - prev_time) <= time_interval)
                {
                    yaw.kp = pid_constants[0] + _h;
                    pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time), false);
                    cost_h[0] += sqrt(subs.yaw_error * subs.yaw_error * dt);
                }
                else if (increasing_h_kp)
                {
                    increasing_h_kp = false;
                    increasing_h_ki = true;
                    cmd_pwm.arm = false;
                    ROS_INFO("Press 'p' to continue the process _h_kp");
                }
                else if (increasing_h_ki && (time_now - prev_time) <= time_interval)
                {
                    yaw.ki = pid_constants[1] + _h;
                    pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time), false);
                    cost_h[1] += sqrt(subs.yaw_error * subs.yaw_error * dt);
                }
                else if (increasing_h_ki)
                {
                    increasing_h_ki = false;
                    increasing_h_kd = true;
                    cmd_pwm.arm = false;
                    ROS_INFO("Press 'p' to continue the process _h_ki");
                }
                else if (increasing_h_kd && (time_now - prev_time) <= time_interval)
                {
                    yaw.kd = pid_constants[2] + _h;
                    pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time), false);
                    cost_h[2] += sqrt(subs.yaw_error * subs.yaw_error * dt);
                }
                else if (increasing_h_kd)
                {
                    increasing_h_kd = false;
                    increasing_h = false;
                    cmd_pwm.arm = false;
                    iterations += 1;
                    pid_constants = descent.execute_adam(pid_constants, cost, cost_h);
                    cost_values.push_back(cost);
                    cost = 0;
                    cost_h[0] = 0;
                    cost_h[1] = 0;
                    cost_h[2] = 0;
                }
            }
            else
            {
                if ((time_now - prev_time) >= time_interval)
                {
                    increasing_h = true;
                    increasing_h_kp = true;
                    cmd_pwm.arm = false;
                }
                else
                {
                    pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time), false);
                    cost += sqrt(subs.yaw_error * subs.yaw_error * dt);
                }
            }
        }

        std_msgs::Float32 cost_msg;
        cost_msg.data = cost;
        cost_publisher.publish(cost_msg);

        cmd_pwm.yaw = pid_yaw;
        pwm_publisher.publish(cmd_pwm);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
