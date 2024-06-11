#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <std_msgs/Char.h>
#include <std_msgs/Float32MultiArray.h>

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
    // yaw.kp                              = 0.4;
    // yaw.ki                              = 0.02;
    // yaw.kd                              = 3.7;
*/

//Predefined Variables
#define threshold 8 //degrees

//Control parameters and PWM Commands
bool forward_bool = false;
custom_msgs::commands   cmd_pwm;
Control     forward, depth, yaw, lateral;

/* Keys callback 
    Function for tuning the PID parameters
*/
void keys_callback(const std_msgs::Char::ConstPtr& msg) {
    char key = msg->data;
    if (key == 'q') {
        cmd_pwm.arm = false;
        std::cout << "unarmed\n";
        forward.emptyError();
        yaw.emptyError();
        depth.emptyError();
        lateral.emptyError();
    }
    else if (key == 'p') {
        cmd_pwm.arm = true;
        std::cout << "armed\n";
    }
    else if (key == 'w') {
        depth.kp = depth.kp+0.01;
        std::cout <<"current depth kp value: "+ std::to_string(depth.kp) << std::endl;
    }
    else if (key == 's') {
        depth.kp = depth.kp-0.01;
        std::cout <<"current depth kp value: "+ std::to_string(depth.kp)<< std::endl;
    }
    else if (key == 'e') {
        depth.ki = depth.ki+0.001;
        std::cout <<"current depth ki value: "+ std::to_string(depth.ki) << std::endl;
    }
    else if (key == 'd') {
        depth.ki = depth.ki-0.001;
        std::cout <<"current depth ki value: "+ std::to_string(depth.ki)<< std::endl;
    }
    else if (key == 'r') {
        depth.kd = depth.kd+0.1;
        std::cout <<"current depth kd value: "+ std::to_string(depth.kd) << std::endl;
    }
    else if (key == 'f') {
        depth.kd = depth.kd-0.1;
        std::cout <<"current depth kd value: "+ std::to_string(depth.kd)<< std::endl;
    }
    else if (key == 't') {
        yaw.kp = yaw.kp+0.005;
        std::cout <<"current yaw kp value: "+ std::to_string(yaw.kp) << std::endl;
    }
    else if (key == 'g') {
        yaw.kp = yaw.kp-0.005;
        std::cout <<"current yaw kp value: "+ std::to_string(yaw.kp)<< std::endl;
    }
    else if (key == 'y') {
        yaw.ki = yaw.ki+0.001;
        std::cout <<"current yaw ki value: "+ std::to_string(yaw.ki) << std::endl;
    }
    else if (key == 'h') {
        yaw.ki = yaw.ki-0.001;
        std::cout <<"current yaw ki value: "+ std::to_string(yaw.ki)<< std::endl;
    }
    else if (key == 'u') {
        yaw.kd = yaw.kd+0.1;
        std::cout <<"current yaw kd value: "+ std::to_string(yaw.kd) << std::endl;
    }
    else if (key == 'j') {
        yaw.kd = yaw.kd-0.1;
        std::cout <<"current yaw kd value: "+ std::to_string(yaw.kd)<< std::endl;
    }
    else if (key == 'm') {
        std::cout <<"Forward switch"<< std::endl;
        forward_bool = !forward_bool;
    }
}

int main(int argc, char **argv) {
    //ROS Node Declaration
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle                     nh;

    //ROS Publisher 
    ros::Publisher pwm_publisher        = nh.advertise<custom_msgs::commands>("/master/commands", 1);

    //ROS Subscriber
    ros::Subscriber keys_subscriber     = nh.subscribe("keys", 1, keys_callback);
    Subscriber                          subs(nh);

    //Control Parameters Definition
    //Yaw
    yaw.kp                              = 0.35;
    yaw.ki                              = 0.027;
    yaw.kd                              = 5.0;

    //Depth
    depth.kp                            = 1;
    depth.ki                            = 0.087;
    depth.kd                            = 14.1;
    // depth.kp                            = 0.913266;
    // depth.ki                            = 0.0668329;
    // depth.kd                            = 12.054;
    // depth.kp                            = 0.83;
    // depth.ki                            = 0.0205;
    // depth.kd                            = 8.6;

    //Forward
    forward.kp                          = 0.125;
    forward.ki                          = 0.0;
    forward.kd                          = 0.15;

    //Lateral
    lateral.kp                          = 0.125;
    lateral.ki                          = 0.0;
    lateral.kd                          = 0.15;

    //Arm Disarm Parameter
    bool arm                            = false;
    ros::Time init_time                 = ros::Time::now();
    cmd_pwm.arm                         = false;

    while (ros::ok()) {
        // if (subs.autonomy_switch==true){
        if (subs.armed==true){
            cmd_pwm.arm                     = subs.armed;
            cmd_pwm.mode                    = "STABILIZE";
            ros::Time time_now              = ros::Time::now();

            if (cmd_pwm.arm==true) {
                float pid_forward           = forward.pid_control(subs.forward_error,(time_now-init_time).toSec(), true);
                float pid_lateral           = lateral.pid_control(subs.lateral_error,(time_now-init_time).toSec(), false);
                float pid_depth             = depth.pid_control(subs.depth_error,(time_now-init_time).toSec(), false);
                float pid_yaw               = 0;

                if (subs.yaw_locked==true) {
                    pid_yaw                 = yaw.pid_control(subs.yaw_error,(time_now-init_time).toSec(), false);
                }
                else {
                    pid_yaw                 = yaw.pid_control(subs.yaw_error,(time_now-init_time).toSec(), true);
                }

                cmd_pwm.forward             = pid_forward;
                cmd_pwm.lateral             = pid_lateral;
                cmd_pwm.thrust              = pid_depth;
                cmd_pwm.yaw                 = pid_yaw;

                //pipeline uncomment
                // if (subs.forward_error==500) {
                //     cmd_pwm.forward         = 1430;
                // }
                // else if (subs.lateral_error==500) {
                //     cmd_pwm.lateral         = 1570;
                // }

                //pipeline comment
                if (subs.depth_external>1225) {
                    cmd_pwm.forward         = 1500;
                    cmd_pwm.lateral         = 1500;
                }
                
                std_msgs::Float32MultiArray v;
            }
            pwm_publisher.publish(cmd_pwm);
        }
        else {
            forward.emptyError();
            yaw.emptyError();
            depth.emptyError();
            lateral.emptyError();
            // pwm_publisher.publish(subs.rov_commands);
        }
        
        ros::spinOnce();
    }
}