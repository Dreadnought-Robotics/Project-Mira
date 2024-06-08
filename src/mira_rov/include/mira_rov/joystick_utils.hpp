#include <ros/ros.h>
#include <custom_msgs/commands.h>
#include <custom_msgs/telemetry.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <../../mira_controller/include/mira_controller/control_utils.hpp>

class controller{
    public:
        Control                  yaw;
        Control                  depth_controller;
        ros::Time                init_time = ros::Time::now();
        ros::Subscriber          joy_sub;
        ros::Subscriber          heading_sub;
        ros::Subscriber          depth_sub;
        ros::Publisher           base_pwm_pub;
        float                    sensitivity;
        int                      prev_msg, prev_yaw_hold=0, prev_depth_hold=0;
        int                      arm_disarm;
        custom_msgs::commands    msg_to_pub;
        bool                     yaw_locked=false, depth_locked=false, autonomy_switch=false;
        ros::ServiceClient       autonomy;
        float                    yaw_err, heading_mark, heading_reading;
        float                    depth_err, depth_mark, depth_reading;
        controller(ros::NodeHandle nh){ 
            joy_sub              = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &controller::joyCallback, this);
            heading_sub          = nh.subscribe<std_msgs::Float32>("/mira/heading", 1, &controller::headingCallback, this);
            depth_sub            = nh.subscribe<custom_msgs::telemetry> ("/master/telemetry", 1 , &controller::depthCallback, this);
            base_pwm_pub         = nh.advertise<custom_msgs::commands>("/rov/commands",1);
            autonomy             = nh.serviceClient<std_srvs::Empty>("/mira/switch");
            msg_to_pub.arm       = 0;
            msg_to_pub.mode      = "STABILIZE";    
            msg_to_pub.forward   = 1500;
            msg_to_pub.lateral   = 1500;
            msg_to_pub.thrust    = 1500;
            msg_to_pub.pitch     = 1500;
            msg_to_pub.yaw       = 1500;
            msg_to_pub.roll      = 1500;
            msg_to_pub.servo1    = 1500;
            msg_to_pub.servo2    = 1500;
            yaw.kp               = 0.35;
            yaw.ki               = 0.027;
            yaw.kd               = 5.0;
            depth_controller.kp  = 1;
            depth_controller.ki  = 0.087;
            depth_controller.kd  = 14.1;
            sensitivity          = 1;

        }
        void depthCallback(const custom_msgs::telemetry::ConstPtr& msg) {
            depth_reading        = msg->external_pressure;
        }
        void headingCallback(const std_msgs::Float32::ConstPtr& msg) {
            heading_reading      = msg->data;
        }
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) { 

        //---------------------------------------------------------------------------    
         
            // int sensitivity_switch = msg->buttons[2];
            int pitch_button       = msg->buttons[2];        
            int mode_stabilise     = msg->buttons[1];
            int mode_acro          = msg->buttons[9];
            int mode_manual        = msg->buttons[3];
            int mode_depth_hold    = msg->buttons[6]; // pixhawk depth hold
            int mira_switch        = msg->buttons[8];
            int yaw_hold_button    = msg->buttons[4];
            int depth_hold_button  = msg->buttons[5]; // custom depth hold
            prev_msg               = arm_disarm;
            arm_disarm             = msg->buttons[0];
            ros::Time time_now     = ros::Time::now();
            msg_to_pub.roll        = 1500+((msg->axes[1])*400);
            msg_to_pub.pitch       = 1500;
            // msg_to_pub.roll=1500+(((msg->buttons[4])*-400)+((msg->buttons[5])*400))*sensitivity;
            msg_to_pub.thrust      = 1500+((((msg->axes[5])+1)*-200)+(((msg->axes[2])+1)*200))*sensitivity;
            msg_to_pub.forward     = 1500+(((msg->axes[4])*400))*sensitivity;
            msg_to_pub.lateral     = 1500+(((msg->axes[3])*-400))*sensitivity;
 
            if (yaw_locked==false) {
                msg_to_pub.yaw     = 1500+((((msg->axes[0])*-400))*sensitivity);
            }
            else {
                yaw_err            = heading_mark - heading_reading;
                float pid_yaw      = yaw.pid_control(yaw_err,(time_now-init_time).toSec(), false);
                msg_to_pub.yaw     = pid_yaw;
            }
            if (depth_locked==false) {
                msg_to_pub.thrust  = 1500+((((msg->axes[5])+1)*-200)+(((msg->axes[2])+1)*200))*sensitivity;
            }
            else {
                depth_err          = depth_mark - depth_reading;
                float pid_thrust   = depth_controller.pid_control(depth_err,(time_now-init_time).toSec(), false);
                msg_to_pub.thrust  = pid_thrust;
            }

        /*

            int sensitivity_switch = msg->buttons[2];        
            int mode_stabilise = msg->buttons[7];
            int mode_acro = msg->buttons[8];
            int mode_manual = msg->buttons[6];
            int mode_depth_hold = msg->buttons[6];
            int mira_switch = msg->buttons[9];
            int yaw_hold_button= msg->buttons[2];
            int depth_hold_button= msg->buttons[3];
            prev_msg = arm_disarm;
            arm_disarm = msg->buttons[10];
            ros::Time time_now              = ros::Time::now();
            msg_to_pub.pitch=1500+(((msg->axes[5])*400)*sensitivity);
            msg_to_pub.roll=1500+((msg->buttons[4]*400)*sensitivity);
            msg_to_pub.thrust=1500+((((msg->axes[3])+1)*-200)+(((msg->axes[2])+1)*200))*sensitivity;
            msg_to_pub.forward=1500+(((msg->axes[1])*400))*sensitivity;
            msg_to_pub.lateral=1500+(((msg->axes[0])*-400))*sensitivity;
            if (yaw_locked==false) {
                msg_to_pub.yaw=1500+((((msg->axes[2])*-400))*sensitivity);
            }
            else {
                yaw_err = heading_mark - heading_reading;
                float pid_yaw                   = yaw.pid_control(yaw_err,(time_now-init_time).toSec(), false);
                msg_to_pub.yaw                  = pid_yaw;
            }
            if (depth_locked==false) {
                msg_to_pub.thrust=1500+((msg->axes[3]*200)*sensitivity);
            }
            else {
                depth_err = depth_mark - depth_reading;
                float pid_thrust                   = depth_controller.pid_control(depth_err,(time_now-init_time).toSec(), false);
                msg_to_pub.thrust                  = pid_thrust;
            }
        */

            if (yaw_hold_button==1 && prev_yaw_hold==0) {
                prev_yaw_hold = 1;
                yaw_locked = !yaw_locked;
                if (yaw_locked==true){
                    heading_mark = heading_reading;
                    ROS_INFO("MARKED YAW=%f",heading_mark);
                    ROS_INFO("Yaw lock Enabled");
                }
                else {
                    ROS_INFO("Yaw lock Disabled");
                }
            }
            else if (yaw_hold_button==0){
                prev_yaw_hold=0;
            }

            if (depth_hold_button==1 && prev_depth_hold==0) {
                prev_depth_hold = 1;
                depth_locked = !depth_locked;
                if (depth_locked==true){
                    depth_mark = depth_reading;
                    ROS_INFO("MARKED DEPTH=%f",depth_mark);
                    ROS_INFO("Depth lock Enabled");
                }
                else {
                    ROS_INFO("Depth lock Disabled");
                }
            }
            else if (depth_hold_button==0){
                prev_depth_hold=0;
            }
            
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
            if (prev_msg==0 && mira_switch==1) {
                std::cout << "TYPE |CONFIRM| TO SWITCH \n";
                std::string typo;
                std::cin >> typo;
                if (typo=="CONFIRM") {
                    std_srvs::Empty s;
                    autonomy.call(s);
                    autonomy_switch=!autonomy_switch;
                    if (autonomy_switch==true) {
                        ROS_INFO("AUTONOMOUS MODE");
                    }
                    else {
                        ROS_INFO("ROV MODE");
                    }
                }
            }
            if (pitch_button==1) {
                msg_to_pub.pitch = 1550;
            }
            else {
                msg_to_pub.pitch = 1500;
            }
            if(mode_stabilise==1 && msg_to_pub.mode!="STABILIZE"){
                msg_to_pub.mode="STABILIZE";
                ROS_INFO("Mode changed to STABILIZE");
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
            // if (autonomy_switch==false){
            //     base_pwm_pub_master.publish(msg_to_pub);
            // }
            // else {
                base_pwm_pub.publish(msg_to_pub);     
            // }
        }
};