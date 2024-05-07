#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>

#define distance_threshold 0.01
#define theta_threshold 0.01

ros::Publisher      error_pub; 
bool step1=true;
int mark_index;

void waypoints_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    int no_of_arucos = msg->data.size()/11;
    float min_distance = 9999, theta1, heading_error, distance_error, forward_error, lateral_error;  
    int closest_aruco, min_index;
    for (int i=0; i<no_of_arucos; i++) {
        float curr_dist = sqrt(msg->data[i*no_of_arucos+2]*msg->data[i*no_of_arucos+2] + msg->data[i*no_of_arucos+3]*msg->data[i*no_of_arucos+3]);
        // if (curr_dist < min_distance) {
        //     min_distance    = curr_dist;
        //     // theta1          = round(((atan2((round(msg->data[i*no_of_arucos+2]*100)/100), (round(msg->data[i*no_of_arucos+1]*100)/100)))* 180 / 3.14)*100)/100;
        //     closest_aruco   = msg->data[i*no_of_arucos];
        //     min_index       = i*no_of_arucos;
        //     forward_error   = msg->data[i*no_of_arucos+2];
        //     lateral_error   = msg->data[i*no_of_arucos+3];
        // }
        if (msg->data[i*no_of_arucos] == 96) {
            min_distance    = curr_dist;
            // theta1          = round(((atan2((round(msg->data[i*no_of_arucos+2]*100)/100), (round(msg->data[i*no_of_arucos+1]*100)/100)))* 180 / 3.14)*100)/100;
            closest_aruco   = msg->data[i*no_of_arucos];
            min_index       = i*no_of_arucos;
            forward_error   = msg->data[i*no_of_arucos+2];
            lateral_error   = msg->data[i*no_of_arucos+3];
            std::cout << forward_error << ", " << lateral_error << std::endl;
        }
        else {
            std::cout << msg->data[i*no_of_arucos] <<std::endl;
        }
    }
    // if (step1 == true) {
    //     if (sqrt(min_distance*min_distance)>=distance_threshold) {
    //         heading_error = 0;
    //     } 
    //     else {
    //         if (closest_aruco == 96) {
    //             heading_error   = 135 - msg->data[min_index+1];
    //         }
    //         else if (closest_aruco == 7) {
    //             heading_error   = 225 - msg->data[min_index+1];
    //         }
    //         else if (closest_aruco == 19) {
    //             heading_error   = 45 - msg->data[min_index+1];
    //         }
    //         else {
    //             heading_error   = 45 + msg->data[min_index+1];
    //         }
    //         if (heading_error<=theta_threshold) {
    //             step1 = false;
    //             mark_index = closest_aruco;
    //         }
    //     }
    // }
    // else {
    //     for (int i=0; i<no_of_arucos; i++) {
    //         if (msg->data[i*no_of_arucos] == mark_index) {
    //             float curr_dist = sqrt(msg->data[i*no_of_arucos+2]*msg->data[i*no_of_arucos+2] + msg->data[i*no_of_arucos+3]*msg->data[i*no_of_arucos+3]);
    //             forward_error = 72.11 - curr_dist;      
    //             break;
    //         }
    //     }
    // }
    geometry_msgs::Quaternion q;
    q.w = 0;//heading_error;
    q.x = forward_error;
    q.y = lateral_error;
    error_pub.publish(q);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "docking_plan");
    ros::NodeHandle                nh;
    ros::Subscriber aruco_subscriber     = nh.subscribe<std_msgs::Float32MultiArray>("/aruco/waypoints", 1, waypoints_callback);
    error_pub               = nh.advertise<geometry_msgs::Quaternion>("/docking/errors", 1);
    ros::spin();
}