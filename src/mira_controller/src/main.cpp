#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <custom_msgs/commands.h>
#include <std_msgs/Char.h>
 
//   +ve   x  -ve (CCW)
//   (CW)  |   
//   y -- bot 
//   +ve
bool forward_bool = false;
#define threshold 8 //degrees
custom_msgs::commands   cmd_pwm;
Control     forward, depth, yaw, lateral;
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
        depth.kp = depth.kp+0.5;
        std::cout <<"current kp value: "+ std::to_string(depth.kp) << std::endl;
    }
    else if (key == 's') {
        depth.kp = depth.kp-0.5;
        std::cout <<"current kp value: "+ std::to_string(depth.kp)<< std::endl;
    }
    else if (key == 'e') {
        depth.kp = depth.ki+0.1;
        std::cout <<"current ki value: "+ std::to_string(depth.ki) << std::endl;
    }
    else if (key == 'd') {
        depth.kp = depth.ki-0.1;
        std::cout <<"current ki value: "+ std::to_string(depth.ki)<< std::endl;
    }
    else if (key == 'r') {
        depth.kp = depth.kd+0.1;
        std::cout <<"current kd value: "+ std::to_string(depth.kd) << std::endl;
    }
    else if (key == 'f') {
        depth.kp = depth.kd-0.1;
        std::cout <<"current kd value: "+ std::to_string(depth.kd)<< std::endl;
    }
    else if (key == 'm') {
        std::cout <<"Forward switch"<< std::endl;
        forward_bool = !forward_bool;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle                     nh;
    ros::Publisher pwm_publisher        = nh.advertise<custom_msgs::commands>("/master/commands", 1);
    ros::Subscriber keys_subscriber     = nh.subscribe("keys", 1, keys_callback);
    Subscriber                          subs(nh);
    yaw.kp                              = 0.77;
    yaw.ki                              = 0.045;
    yaw.kd                              = 0;
    depth.kp                            = 1.35;
    depth.ki                            = 0.266;
    depth.kd                            = 0.335;
    forward.kp                          = 3;
    forward.ki                          = 0;
    forward.kd                          = 0;
    lateral.kp                          = 3;
    lateral.ki                          = 0;
    lateral.kd                          = 0;
    bool arm                            = true;
    ros::Time init_time                 = ros::Time::now();
    cmd_pwm.arm                         = true;
    while (ros::ok()) {
        cmd_pwm.mode                    = "STABILIZE";
        ros::Time time_now              = ros::Time::now();
        if (!forward.error_vector.empty()) {
            auto it                     = forward.error_vector.end();
            if (*it - subs.forward_error > threshold) {
                int num_elements        = std::min(static_cast<int>(forward.error_vector.size()), 10);
                auto max_element_iter   = std::max_element(forward.error_vector.end() - num_elements, forward.error_vector.end());
                subs.forward_error      = *max_element_iter;
            }
            else if(*it - subs.forward_error < -1*threshold) {
                int num_elements        = std::min(static_cast<int>(forward.error_vector.size()), 10);
                auto min_element_iter   = std::min_element(forward.error_vector.end() - num_elements, forward.error_vector.end());
                subs.forward_error      = *min_element_iter;
            }
        }
        if (!lateral.error_vector.empty()) {
            auto it                     = lateral.error_vector.end();
            if (*it - subs.lateral_error > threshold) {
                int num_elements        = std::min(static_cast<int>(lateral.error_vector.size()), 10);
                auto max_element_iter   = std::max_element(lateral.error_vector.end() - num_elements, lateral.error_vector.end());
                subs.lateral_error      = *max_element_iter;
            }
            else if (*it - subs.lateral_error < -1*threshold) {
                int num_elements        = std::min(static_cast<int>(lateral.error_vector.size()), 10);
                auto min_element_iter   = std::min_element(lateral.error_vector.end() - num_elements, lateral.error_vector.end());
                subs.lateral_error      = *min_element_iter;
            }
        }
        float pid_forward               = forward.pid_control(subs.forward_error, (time_now-init_time).toSec(), true);
        float pid_lateral               = lateral.pid_control(subs.lateral_error, (time_now-init_time).toSec(), true);
        float pid_depth                 = depth.pid_control(subs.depth_error, (time_now-init_time).toSec(), false);
        float pid_yaw                   = yaw.pid_control(subs.yaw_error, (time_now-init_time).toSec(), true);
        if (sqrt(pow(subs.forward_error,2))>threshold) {
            if (forward_bool) {
                cmd_pwm.forward         = pid_forward;
                cmd_pwm.lateral         = pid_lateral;
            }
            else {
                cmd_pwm.forward         = 0;
            }
            cmd_pwm.thrust              = pid_depth;
            cmd_pwm.yaw                 = pid_yaw;
        }
        else {
            if (forward_bool) {
                cmd_pwm.forward         = pid_forward;
                cmd_pwm.lateral         = pid_lateral;
            }
            else {
                cmd_pwm.forward         = 0;
            }
            cmd_pwm.thrust              = pid_depth;
            cmd_pwm.yaw                 = pid_yaw;
        }
        pwm_publisher.publish(cmd_pwm);
        ros::spinOnce();
    }
}