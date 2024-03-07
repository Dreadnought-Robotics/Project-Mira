#include<rov_joystick/joystick_utils.hpp>
#include <ros/ros.h>
#include<custom_msgs/commands.h>


//run the joy node with:
// rosrun joy joy_node _autorepeat_rate:=1000 --dev /dev/input/js0 
int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_controller_node");
    ros::NodeHandle nh;
    controller obj= controller(nh);
    
    while(ros::ok()){
        ros::spinOnce();
    }
}
