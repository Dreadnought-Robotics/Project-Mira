#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <custom_msgs/commands.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <string>
using namespace std;

/*      
    Bot Orientation
    +ve   x  -ve (CCW)
    (CW)  |   
    y -- bot 
    +ve
*/
/*
    Old Tuning
    yaw.kp                              = 0.77;
    yaw.ki                              = 0.045;
    yaw.kd                              = 0;
    depth.kp                            = 1.35;
    depth.ki                            = 0.266;
    depth.kd                            = 0.335;
*/

constexpr float threshold = 8.0f; // degrees
custom_msgs::commands cmd_pwm;
Control forward, depth, yaw, lateral;
bool tuning_mode = false;
char tuning_axis = '\0'; // 'd' for depth, 'y' for yaw, 'f' for forward, 'l' for lateral

void keys_callback(const std_msgs::Char::ConstPtr& msg) {
    char key = msg->data;
    if (key == 'a') {
        cmd_pwm.arm = true;
        cout<<"Mira Armed"<<endl;
    } 
    else if (key == 'z') {
        cmd_pwm.arm = false;
        cout<<"Mira Disarmed"<<endl;
    }
    else if (key == 'q') {
        tuning_mode = false;
        tuning_axis = '\0';
        cout<<"--------PID values for each axis--------"<<endl;
        cout<<"Depth PID: "<<depth.kp<<" "<<depth.ki<<" "<<depth.kd<<endl;
        cout<<"Forward PID: "<<forward.kp<<" "<<forward.ki<<" "<<forward.kd<<endl;
        cout<<"Lateral PID: "<<lateral.kp<<" "<<lateral.ki<<" "<<lateral.kd<<endl;
        cout<<"Yaw PID: "<<yaw.kp<<" "<<yaw.ki<<" "<<yaw.kd<<endl;        
    } 
    else if (key == 'd') {
        tuning_mode = true;
        tuning_axis = 'd';
        cout<<"Tuning depth PID"<<endl;
    } 
    else if (key == 'y') {
        tuning_mode = true;
        tuning_axis = 'y';
        cout<<"Tuning yaw PID"<<endl;
    } 
    else if (key == 'f') {
        tuning_mode = true;
        tuning_axis = 'f';
        cout<<"Tuning forward PID"<<endl;
    }
    else if (key == 'l') {
        tuning_mode = true;
        tuning_axis = 'l';
        cout<<"Tuning lateral PID"<<endl;
    } 
    else if (tuning_mode) {
        float val = stof(string(1, key));
        switch (tuning_axis) {
            case 'd':
                if (key == '1') depth.kp += val;
                else if (key == '2') depth.ki += val;
                else if (key == '3') depth.kd += val;
                cout<<"Current depth PID values: "<<depth.kp<<" "<<depth.ki<<" "<<depth.kd<<endl;
                break;
            case 'y':
                if (key == '1') yaw.kp += val;
                else if (key == '2') yaw.ki += val;
                else if (key == '3') yaw.kd += val;
                cout<<"Current yaw PID values: "<<yaw.kp<<" "<<yaw.ki<<" "<<yaw.kd<<endl;
                break;
            case 'f':
                if (key == '1') forward.kp += val;
                else if (key == '2') forward.ki += val;
                else if (key == '3') forward.kd += val;
                cout<<"Current forward PID values: "<<forward.kp<<" "<<forward.ki<<" "<<forward.kd<<endl;
                break;
            case 'l':
                if (key == '1') lateral.kp += val;
                else if (key == '2') lateral.ki += val;
                else if (key == '3') lateral.kd += val;
                cout<<"Current lateral PID values: "<<lateral.kp<<" "<<lateral.ki<<" "<<lateral.kd<<endl;
                break;
        }
    } 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;
    ros::Publisher pwm_publisher = nh.advertise<custom_msgs::commands>("/master/commands", 1);
    ros::Subscriber keys_subscriber = nh.subscribe("keys", 1, keys_callback);

    Subscriber subs(nh);
    yaw = {2.6, 0.023, 1.5};
    depth = {1.2, 0.036, 1.2};
    forward = {0.2, 0.0, 2.5};
    lateral = {0.15, 0.1, 0.5};
    cmd_pwm.arm = false;
    ros::Time init_time = ros::Time::now();

    while (ros::ok()) {
        cmd_pwm.mode = "STABILIZE";
        ros::Time time_now = ros::Time::now();
        float pid_forward = forward.pid_control(subs.forward_error, (time_now - init_time).toSec(), true);
        float pid_lateral = lateral.pid_control(subs.lateral_error, (time_now - init_time).toSec(), false);
        float pid_depth = depth.pid_control(subs.depth_error, (time_now - init_time).toSec(), false);
        float pid_yaw = yaw.pid_control(subs.yaw_error, (time_now - init_time).toSec(), true);

        float forward_error_sq = pow(subs.forward_error, 2);
        if (forward_error_sq > threshold * threshold) {
            cmd_pwm.forward = pid_forward;
            cmd_pwm.lateral = pid_lateral;
            cmd_pwm.thrust = pid_depth;
            cmd_pwm.yaw = pid_yaw;
        } else {
            cmd_pwm.forward = cmd_pwm.forward ? pid_forward : 0;
            cmd_pwm.lateral = cmd_pwm.forward ? pid_lateral : 0;
            cmd_pwm.thrust = pid_depth;
            cmd_pwm.yaw = pid_yaw;
        }

        if (subs.depth_service_called) {
            cmd_pwm.thrust = 1450;
        }
        cmd_pwm.yaw = 1500;

        pwm_publisher.publish(cmd_pwm);
        ros::spinOnce();
    }
    return 0;
}