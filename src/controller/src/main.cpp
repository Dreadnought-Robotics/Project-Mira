#include <ros/ros.h>
#include <controller/control_utils.hpp>
#include <controller/subs_utils.hpp>
#include <custom_msgs/commands.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>

 
//   +ve   x  -ve (CCW)
//   (CW)  |   
//   y -- bot 
//   +ve

#define threshold 8 //degrees

int main(int argc, char **argv) {

    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;
    ros::Publisher pwm_publisher = nh.advertise<custom_msgs::commands>("/master/commands", 1);
    
    Subscriber  subs(nh);
    Control     yaw, forward;
    yaw.kp      = 3.69;
    yaw.ki      = 0;
    yaw.kd      = 0;

    ros::Time init_time = ros::Time::now();
    while (ros::ok()) {
        custom_msgs::commands   cmd_pwm;
        ros::Time time_now      = ros::Time::now();
        float pid_yaw           = yaw.pid_control(subs.yaw_error, (time_now-init_time).toSec());
        if (sqrt(pow(subs.yaw_error,2))>threshold) {
            cmd_pwm.yaw         = pid_yaw;
        }
        else {
            cmd_pwm.yaw         = yaw.hold();
        }
        pwm_publisher.publish(cmd_pwm);
        ros::spinOnce();
    }
}