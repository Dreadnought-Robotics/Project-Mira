#include <ros/ros.h>
#include <custom_msgs/commands.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
using namespace std;

class controller{
    public:
        ros::Subscriber joy_sub;
        ros::Publisher base_pwm_pub;
        int prev_msg;
        int arm_disarm;
        custom_msgs::commands msg_to_pub;
        controller(ros::NodeHandle nh){ 
            joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &controller::joyCallback, this);
            base_pwm_pub = nh.advertise<custom_msgs::commands>("/master/commands",1);
            msg_to_pub.arm=0;
            msg_to_pub.mode="STABILIZE";    
            msg_to_pub.forward=1500;
            msg_to_pub.lateral=1500;
            msg_to_pub.thrust=1500;
            msg_to_pub.pitch=1500;
            msg_to_pub.yaw=1500;
            msg_to_pub.roll=1500;
            msg_to_pub.servo1=1500;
            msg_to_pub.servo2=1500;
        }

        int map_to_pwm(float thrust){
            float thrust_vals[201]={-3.795, -3.775, -3.755, -3.705, -3.65, -3.615, -3.5549999999999997, -3.525, -3.4699999999999998, -3.425, -3.38, -3.325, -3.26, -3.1950000000000003, -3.1399999999999997, -3.08, -3.0149999999999997, -2.98, -2.9, -2.84, -2.79, -2.735, -2.675, -2.635, -2.59, -2.5300000000000002, -2.5, -2.42, -2.375, -2.3449999999999998, -2.285, -2.215, -2.19, -2.135, -2.075, -2.045, -1.9700000000000002, -1.92, -1.87, -1.83, -1.785, -1.7200000000000002, -1.685, -1.6400000000000001, -1.5899999999999999, -1.555, -1.505, -1.465, -1.415, -1.385, -1.3399999999999999, -1.295, -1.26, -1.21, -1.175, -1.125, -1.0899999999999999, -1.05, -1.025, -0.9850000000000001, -0.95, -0.915, -0.875, -0.835, -0.8049999999999999, -0.76, -0.73, -0.6950000000000001, -0.6699999999999999, -0.635, -0.6000000000000001, -0.5700000000000001, -0.5349999999999999, -0.5, -0.47, -0.44, -0.405, -0.385, -0.355, -0.31999999999999995, -0.29000000000000004, -0.265, -0.235, -0.215, -0.185, -0.16499999999999998, -0.135, -0.115, -0.09, -0.08, -0.060000000000000005, -0.04, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.05, 0.07, 0.09, 0.115, 0.135, 0.16499999999999998, 0.2, 0.225, 0.26, 0.29000000000000004, 0.32499999999999996, 0.365, 0.4, 0.43, 0.47, 0.51, 0.55, 0.585, 0.63, 0.6699999999999999, 0.71, 0.7549999999999999, 0.8, 0.8400000000000001, 0.8899999999999999, 0.9199999999999999, 0.96, 1.01, 1.055, 1.0899999999999999, 1.145, 1.185, 1.2349999999999999, 1.295, 1.335, 1.38, 1.4300000000000002, 1.48, 1.5299999999999998, 1.575, 1.635, 1.685, 1.745, 1.79, 1.85, 1.9049999999999998, 1.965, 2.015, 2.065, 2.12, 2.19, 2.26, 2.3200000000000003, 2.37, 2.45, 2.5, 2.55, 2.635, 2.6950000000000003, 2.77, 2.8449999999999998, 2.885, 2.965, 3.0, 3.09, 3.1500000000000004, 3.205, 3.28, 3.37, 3.435, 3.4850000000000003, 3.57, 3.625, 3.69, 3.775, 3.8600000000000003, 3.935, 3.995, 4.075, 4.175, 4.225, 4.315, 4.37, 4.45, 4.495, 4.57, 4.654999999999999, 4.73, 4.779999999999999, 4.845, 4.875, 4.885};
            int pwm = 0;
            for(int i=0; i<200; i++){
                if((thrust>=thrust_vals[i])&&(thrust<=thrust_vals[i+1])){
                    pwm=(1100+(i*4))+(((thrust-thrust_vals[i])/(thrust_vals[i+1]-thrust_vals[i]))*4);
                    break;
                }  
            }
            return pwm;
        }

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {        
            int mode_stabilise=msg->buttons[1];
            int mode_acro=msg->buttons[4];
            int mode_manual=msg->buttons[3];
            int mode_depth_hold=msg->buttons[2];
            prev_msg=arm_disarm;
            arm_disarm = msg->buttons[0];

            /*
            msg_to_pub.pitch=1500+((msg->axes[1])*400);
            msg_to_pub.roll=1500+((msg->buttons[4])*-400)+((msg->buttons[5])*400);
            msg_to_pub.thrust=1500+(((msg->axes[5])+1)*-200)+(((msg->axes[2])+1)*200);
            msg_to_pub.yaw=1500+((msg->axes[0])*-400);
            msg_to_pub.forward=1500+((msg->axes[4])*400);
            msg_to_pub.lateral=1500+((msg->axes[3])*-400);
            */
            //--------------------------------------
            
            msg_to_pub.pitch=map_to_pwm((msg->axes[1])*3.7);
            msg_to_pub.roll=map_to_pwm((msg->buttons[4])*-3.7)+((msg->buttons[5])*3.7);
            msg_to_pub.thrust=map_to_pwm(((msg->axes[5])+1)*-1.85)+(((msg->axes[2])+1)*1.85);
            msg_to_pub.yaw=map_to_pwm((msg->axes[0])*-3.7);
            msg_to_pub.forward=map_to_pwm((msg->axes[4])*3.7);
            msg_to_pub.lateral=map_to_pwm((msg->axes[3])*-3.7);

            if(arm_disarm==1 && prev_msg==0){
                if(msg_to_pub.arm==0){
                    msg_to_pub.arm=1;
                    ROS_WARN("VEHICLE ARMED");
                }
                else{
                    msg_to_pub.arm=0;
                    ROS_WARN("VEHICLE DISARMED");
                }
            }
            if(mode_stabilise==1 && msg_to_pub.mode!="STABILIZE"){
                msg_to_pub.mode="STABILIZE";
                ROS_INFO("Mode changed to STABILIZE ");
            }
            if(mode_acro==1  && msg_to_pub.mode!="ACRO"){
                msg_to_pub.mode="ACRO";
                ROS_INFO("Mode changed to ACRO ");
            } 
            if(mode_manual==1  && msg_to_pub.mode!="MANUAL"){
                msg_to_pub.mode="MANUAL";
                ROS_INFO("Mode changed to MANUAL ");
            }
            if(mode_depth_hold==1  && msg_to_pub.mode!="ALT_HOLD"){
                msg_to_pub.mode="ALT_HOLD";
                ROS_INFO("Mode changed to ALT_HOLD ");
            }
            base_pwm_pub.publish(msg_to_pub);       
        }
};
