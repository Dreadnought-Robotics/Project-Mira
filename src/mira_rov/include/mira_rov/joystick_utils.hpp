#include <ros/ros.h>
#include <custom_msgs/commands.h>
#include <sensor_msgs/Joy.h>


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

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {        
            int mode_stabilise=msg->buttons[1];
            int mode_acro=msg->buttons[4];
            int mode_manual=msg->buttons[3];
            int mode_depth_hold=msg->buttons[2];
            prev_msg=arm_disarm;
            arm_disarm = msg->buttons[0];

            msg_to_pub.pitch=1500+((msg->axes[1])*400);
            msg_to_pub.roll=1500+((msg->buttons[4])*-400)+((msg->buttons[5])*400);
            msg_to_pub.thrust=1500+(((msg->axes[5])+1)*-200)+(((msg->axes[2])+1)*200);
            msg_to_pub.yaw=1500+((msg->axes[0])*-400);

            msg_to_pub.forward=1500+((msg->axes[4])*400);
            msg_to_pub.lateral=1500+((msg->axes[3])*-400);

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