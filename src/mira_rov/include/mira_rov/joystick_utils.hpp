#include <ros/ros.h>
#include <custom_msgs/commands.h>
#include <custom_msgs/telemetry.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <../../mira_controller/include/mira_controller/control_utils.hpp>

class Controller
{
public:
    Control yaw;
    Control depth_controller;
    ros::Time init_time;
    ros::Subscriber joy_sub;
    ros::Subscriber heading_sub;
    ros::Subscriber depth_sub;
    ros::Publisher base_pwm_pub;
    ros::ServiceClient autonomy;

    float sensitivity;
    int prev_msg, prev_yaw_hold = 0, prev_depth_hold = 0;
    int arm_disarm;
    custom_msgs::commands msg_to_pub;
    bool yaw_locked = false;
    bool depth_locked = false;
    bool autonomy_switch = false;
    float yaw_err, heading_mark, heading_reading;
    float depth_err, depth_mark, depth_reading;
    /**
     * @brief Constructor initializes subscribers, publishers, and service clients.
     * @param nh ROS node handle
     */
    Controller(ros::NodeHandle nh)
    {
        joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &Controller::joyCallback, this);
        heading_sub = nh.subscribe<std_msgs::Float32>("/mira/heading", 1, &Controller::headingCallback, this);
        depth_sub = nh.subscribe<custom_msgs::telemetry>("/master/telemetry", 1, &Controller::depthCallback, this);
        base_pwm_pub = nh.advertise<custom_msgs::commands>("/rov/commands", 1);
        autonomy = nh.serviceClient<std_srvs::Empty>("/mira/switch");

        // Initialize message to publish
        msg_to_pub.arm = 0;
        msg_to_pub.mode = "STABILIZE";
        msg_to_pub.forward = 1500;
        msg_to_pub.lateral = 1500;
        msg_to_pub.thrust = 1500;
        msg_to_pub.pitch = 1500;
        msg_to_pub.yaw = 1500;
        msg_to_pub.roll = 1500;
        msg_to_pub.servo1 = 1500;
        msg_to_pub.servo2 = 1500;

        // Initialize control parameters
        yaw.kp = 0.35;
        yaw.ki = 0.027;
        yaw.kd = 5.0;
        depth_controller.kp = 1;
        depth_controller.ki = 0.087;
        depth_controller.kd = 14.1;

        sensitivity = 0.6;
    }

    /**
     * @brief Callback function for depth sensor data.
     * @param msg Depth telemetry message
     */
    void depthCallback(const custom_msgs::telemetry::ConstPtr &msg)
    {
        depth_reading = msg->external_pressure;
    }

    /**
     * @brief Callback function for heading data.
     * @param msg Heading message
     */
    void headingCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        heading_reading = msg->data;
    }

    /**
     * @brief Callback function for joystick inputs.
     * @param msg Joystick message
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
    {
        int pitch_button = msg->buttons[2];
        int mode_stabilize = msg->buttons[1];
        int mode_acro = msg->buttons[7];
        int mode_manual = msg->buttons[3];
        int mode_depth_hold = msg->buttons[6];
        int mira_switch = msg->buttons[8];
        int yaw_hold_button = msg->buttons[4];
        int depth_hold_button = msg->buttons[5];
        prev_msg = arm_disarm;
        arm_disarm = msg->buttons[0];
        ros::Time time_now = ros::Time::now();

        // Control adjustments based on joystick inputs
        msg_to_pub.roll = 1500 + (msg->axes[1] * 200);
        msg_to_pub.pitch = 1500;
        msg_to_pub.thrust = 1500 + (((msg->axes[5] + 1) * -200) + (((msg->axes[2] + 1) * 200))) * sensitivity;
        msg_to_pub.forward = 1500 + ((msg->axes[4] * 400)) * sensitivity;
        msg_to_pub.lateral = 1500 + ((msg->axes[3] * -400)) * sensitivity;

        // Yaw control
        if (!yaw_locked)
        {
            msg_to_pub.yaw = 1500 + ((msg->axes[0] * -400)) * sensitivity;
        }
        else
        {
            yaw_err = heading_mark - heading_reading;
            float pid_yaw = yaw.pid_control(yaw_err, (time_now - init_time).toSec(), false);
            msg_to_pub.yaw = pid_yaw;
        }

        // Depth control
        if (!depth_locked)
        {
            msg_to_pub.thrust = 1500 + (((msg->axes[5] + 1) * -200) + (((msg->axes[2] + 1) * 200))) * sensitivity;
        }
        else
        {
            depth_err = depth_mark - depth_reading;
            float pid_thrust = depth_controller.pid_control(depth_err, (time_now - init_time).toSec(), false);
            msg_to_pub.thrust = pid_thrust;
        }

        // Handle yaw hold button press
        if (yaw_hold_button == 1 && prev_yaw_hold == 0)
        {
            prev_yaw_hold = 1;
            yaw_locked = !yaw_locked;
            if (yaw_locked)
            {
                heading_mark = heading_reading;
                ROS_INFO("MARKED YAW=%f", heading_mark);
                ROS_INFO("Yaw lock Enabled");
            }
            else
            {
                ROS_INFO("Yaw lock Disabled");
            }
        }
        else if (yaw_hold_button == 0)
        {
            prev_yaw_hold = 0;
        }

        // Handle depth hold button press
        if (depth_hold_button == 1 && prev_depth_hold == 0)
        {
            prev_depth_hold = 1;
            depth_locked = !depth_locked;
            if (depth_locked)
            {
                depth_mark = depth_reading;
                ROS_INFO("MARKED DEPTH=%f", depth_mark);
                ROS_INFO("Depth lock Enabled");
            }
            else
            {
                ROS_INFO("Depth lock Disabled");
            }
        }
        else if (depth_hold_button == 0)
        {
            prev_depth_hold = 0;
        }

        // Handle arm/disarm button press
        if (arm_disarm == 1 && prev_msg == 0)
        {
            msg_to_pub.arm = (msg_to_pub.arm == 0) ? 1 : 0;
            if (msg_to_pub.arm == 1)
            {
                ROS_WARN("VEHICLE ARMED");
            }
            else
            {
                ROS_WARN("VEHICLE DISARMED");
            }
        }

        // Handle autonomy switch
        if (prev_msg == 0 && mira_switch == 1)
        {
            std::cout << "TYPE |CONFIRM| TO SWITCH \n";
            std::string typo;
            std::cin >> typo;
            if (typo == "CONFIRM")
            {
                std_srvs::Empty srv;
                autonomy.call(srv);
                autonomy_switch = !autonomy_switch;
                ROS_INFO(autonomy_switch ? "AUTONOMOUS MODE" : "ROV MODE");
            }
        }

        // Handle pitch button press
        msg_to_pub.pitch = (pitch_button == 1) ? 1400 : 1500;

        // Handle mode change
        if (mode_stabilize == 1 && msg_to_pub.mode != "STABILIZE")
        {
            msg_to_pub.mode = "STABILIZE";
            ROS_INFO("Mode changed to STABILIZE");
        }
        if (mode_acro == 1 && msg_to_pub.mode != "ACRO")
        {
            msg_to_pub.mode = "ACRO";
            ROS_INFO("Mode changed to ACRO");
        }
        if (mode_manual == 1 && msg_to_pub.mode != "MANUAL")
        {
            msg_to_pub.mode = "MANUAL";
            ROS_INFO("Mode changed to MANUAL");
        }
        if (mode_depth_hold == 1 && msg_to_pub.mode != "ALT_HOLD")
        {
            msg_to_pub.mode = "ALT_HOLD";
            ROS_INFO("Mode changed to ALT_HOLD");
        }

        base_pwm_pub.publish(msg_to_pub);
    }
};
