#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <custom_msgs/commands.h>
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
*/


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
        forward.kp = forward.kp+0.005;
        std::cout <<"current forward kp value: "+ std::to_string(forward.kp) << std::endl;
    }
    else if (key == 's') {
        forward.kp = forward.kp-0.005;
        std::cout <<"current forward kp value: "+ std::to_string(forward.kp)<< std::endl;
    }
    else if (key == 'e') {
        forward.ki = forward.ki+0.001;
        std::cout <<"current forward ki value: "+ std::to_string(forward.ki) << std::endl;
    }
    else if (key == 'd') {
        forward.ki = forward.ki-0.001;
        std::cout <<"current forward ki value: "+ std::to_string(forward.ki)<< std::endl;
    }
    else if (key == 'r') {
        forward.kd = forward.kd+0.1;
        std::cout <<"current forward kd value: "+ std::to_string(forward.kd) << std::endl;
    }
    else if (key == 'f') {
        forward.kd = forward.kd-0.1;
        std::cout <<"current forward kd value: "+ std::to_string(forward.kd)<< std::endl;
    }
    else if (key == 't') {
        lateral.kp = lateral.kp+0.005;
        std::cout <<"current lateral kp value: "+ std::to_string(lateral.kp) << std::endl;
    }
    else if (key == 'g') {
        lateral.kp = lateral.kp-0.005;
        std::cout <<"current lateral kp value: "+ std::to_string(lateral.kp)<< std::endl;
    }
    else if (key == 'y') {
        lateral.ki = lateral.ki+0.001;
        std::cout <<"current lateral ki value: "+ std::to_string(lateral.ki) << std::endl;
    }
    else if (key == 'h') {
        lateral.ki = lateral.ki-0.001;
        std::cout <<"current lateral ki value: "+ std::to_string(lateral.ki)<< std::endl;
    }
    else if (key == 'u') {
        lateral.kd = lateral.kd+0.1;
        std::cout <<"current lateral kd value: "+ std::to_string(lateral.kd) << std::endl;
    }
    else if (key == 'j') {
        lateral.kd = lateral.kd-0.1;
        std::cout <<"current lateral kd value: "+ std::to_string(lateral.kd)<< std::endl;
    }
    else if (key == 'm') {
        std::cout <<"Forward switch"<< std::endl;
        forward_bool = !forward_bool;
    }
}
// bool tuning_mode = false;
// char tuning_axis = '\0';
// void keys_callback(const std_msgs::Char::ConstPtr& msg) {
    
// char key = msg->data;
//     if (key == 'a') {
//         cmd_pwm.arm = true;
//         std::cout<<"Mira Armed"<<std::endl;
//     } 
//     else if (key == 'z') {
//         cmd_pwm.arm = false;
//         depth.emptyError();
//         std::cout<<"Mira Disarmed"<<std::endl;
//     }
//     else if (key == 'q') {
//         tuning_mode = false;
//         tuning_axis = '\0';
//         std::cout<<"--------PID values for each axis--------"<<std::endl;
//         std::cout<<"Depth PID: "<<depth.kp<<","<<depth.ki<<","<<depth.kd<<std::endl;
//         std::cout<<"Forward PID: "<<forward.kp<<","<<forward.ki<<","<<forward.kd<<std::endl;
//         std::cout<<"Lateral PID: "<<lateral.kp<<","<<lateral.ki<<","<<lateral.kd<<std::endl;
//         std::cout<<"Yaw PID: "<<yaw.kp<<","<<yaw.ki<<","<<yaw.kd<<std::endl;        
//     } 
//     else if (key == 'd') {
//         tuning_mode = true;
//         tuning_axis = 'd';
//         std::cout<<"Tuning depth PID"<<std::endl;
//     } 
//     else if (key == 'y') {
//         tuning_mode = true;
//         tuning_axis = 'y';
//         std::cout<<"Tuning yaw PID"<<std::endl;
//     } 
//     else if (key == 'f') {
//         tuning_mode = true;
//         tuning_axis = 'f';
//         std::cout<<"Tuning forward PID"<<std::endl;
//     }
//     else if (key == 'l') {
//         tuning_mode = true;
//         tuning_axis = 'l';
//         std::cout<<"Tuning lateral PID"<<std::endl;
//     } 
//     else if (tuning_mode) {
//         float val = std::atof(&key);
//         switch (tuning_axis) {
//             case 'd':
//                 if (key == '1') depth.kp += val;
//                 else if (key == '2') depth.ki += val;
//                 else if (key == '3') depth.kd += val;
//                 std::cout<<"Current depth PID values: "<<depth.kp<<" "<<depth.ki<<" "<<depth.kd<<std::endl;
//                 break;
//             case 'y':
//                 if (key == '1') yaw.kp += val;
//                 else if (key == '2') yaw.ki += val;
//                 else if (key == '3') yaw.kd += val;
//                 std::cout<<"Current yaw PID values: "<<yaw.kp<<" "<<yaw.ki<<" "<<yaw.kd<<std::endl;
//                 break;
//             case 'f':
//                 if (key == '1') forward.kp += val;
//                 else if (key == '2') forward.ki += val;
//                 else if (key == '3') forward.kd += val;
//                 std::cout<<"Current forward PID values: "<<forward.kp<<" "<<forward.ki<<" "<<forward.kd<<std::endl;
//                 break;
//             case 'l':
//                 if (key == '1') lateral.kp += val;
//                 else if (key == '2') lateral.ki += val;
//                 else if (key == '3') lateral.kd += val;
//                 std::cout<<"Current lateral PID values: "<<lateral.kp<<" "<<lateral.ki<<" "<<lateral.kd<<std::endl;
//                 break;
//         }
//     } 
// }
int main(int argc, char **argv) {
    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle                     nh;
    ros::Publisher pwm_publisher        = nh.advertise<custom_msgs::commands>("/master/commands", 1);
    ros::Subscriber keys_subscriber     = nh.subscribe("keys", 1, keys_callback);

    Subscriber                          subs(nh);
    yaw.kp                              = 0.4;
    yaw.ki                              = 0.02;
    yaw.kd                              = 3.7;
    depth.kp                            = 0.93;
    depth.ki                            = 0.087;
    depth.kd                            = 12.1;
    forward.kp                          = 0.125;
    forward.ki                          = 0.0;
    forward.kd                          = 0.15;
    lateral.kp                          = 0.125;
    lateral.ki                          = 0.0;
    lateral.kd                          = 0.15;
    bool arm                            = false;
    ros::Time init_time                 = ros::Time::now();
    cmd_pwm.arm                         = false;
    while (ros::ok()) {
        cmd_pwm.mode                    = "STABILIZE";
        // std::cout << "Error: " << subs.depth_error << std::endl;
        ros::Time time_now              = ros::Time::now();
        if (cmd_pwm.arm==true) {
            float pid_forward               = forward.pid_control(subs.forward_error,(time_now-init_time).toSec(), true);
            float pid_lateral               = lateral.pid_control(subs.lateral_error,(time_now-init_time).toSec(), false);
            float pid_depth                 = depth.pid_control(subs.depth_error,(time_now-init_time).toSec(), false);
            float pid_yaw                   = yaw.pid_control(subs.yaw_error,(time_now-init_time).toSec(), true);
            // if (subs.service_called==true) {
            //     yaw.kp = 0.2;
            //     yaw.ki = 0;
            //     yaw.kd = 0;
            // }
            // std::cout << subs.yaw_error << std::endl;
            if (sqrt(pow(subs.forward_error,2))>threshold) {
                // if (forward_bool) {
                    // cmd_pwm.forward         = pid_forward;
                    // cmd_pwm.lateral         = pid_lateral;
                // }
                // else {
                    cmd_pwm.forward         = pid_forward;
                    cmd_pwm.lateral         = pid_lateral;
                // }
                cmd_pwm.thrust              = pid_depth;
                cmd_pwm.yaw                 = pid_yaw;
            }
            else {
            //     if (forward_bool) {
                    // cmd_pwm.forward         = pid_forward;
                    // cmd_pwm.lateral         = pid_lateral;
            //     }
            //     else {
                    cmd_pwm.forward         = pid_forward;
                    cmd_pwm.lateral         = pid_lateral;
            //     }
                cmd_pwm.thrust              = pid_depth;
                cmd_pwm.yaw                 = pid_yaw;
            }
            // if (subs.depth_service_called==true) {
            //     cmd_pwm.thrust = 1450;
            // }
                cmd_pwm.yaw                 = 1500;
                // cmd_pwm.thrust                 = 1500;
                cmd_pwm.forward                 = 1570;
                // cmd_pwm.lateral                 = 1500;
            std_msgs::Float32MultiArray v;
        }
        pwm_publisher.publish(cmd_pwm);
        ros::spinOnce();
    }
}