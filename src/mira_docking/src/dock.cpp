#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#define distance_threshold 0.01
#define theta_threshold 0.1

/*
640*480
-1*y, -1*x
-1*y+240, -1*x+320
*/

class Docking24 {
    public:
        Docking24(ros::NodeHandle nh) {
            yaw_lock                = nh.serviceClient<std_srvs::Empty>("/yaw/lock");
            center_lock             = nh.advertiseService("/center/lock", &Docking24::emptyServiceCallback, this);
            pose3d_subscriber       = nh.subscribe<std_msgs::Float32MultiArray>("/aruco/waypoints", 1, &Docking24::waypoints_callback, this);
            aruco_subscriber        = nh.subscribe<std_msgs::Float32MultiArray>("/aruco/pixels", 1, &Docking24::pixel_callback, this);
            center_subscriber       = nh.subscribe<geometry_msgs::Vector3>("/docking/center", 1, &Docking24::center_callback, this);
            error_pub               = nh.advertise<geometry_msgs::Quaternion>("/docking/errors", 1);
            error_pub_3d            = nh.advertise<geometry_msgs::Quaternion>("/docking/errors/3d", 1);
            ros::spin();
        }

    private:
        ros::Publisher      error_pub;
        ros::Publisher      error_pub_3d;
        ros::ServiceClient  yaw_lock;
        ros::Subscriber     pose3d_subscriber;
        ros::Subscriber     aruco_subscriber;
        ros::Subscriber     center_subscriber;
        ros::ServiceServer  center_lock;
        bool yaw_locked     = false;
        bool center_called  = false;
        void waypoints_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
            int no_of_arucos = msg->data.size()/11;
            float min_distance = 9999, theta1, heading_error, distance_error, forward_error, lateral_error;  
            int closest_aruco, min_index;
            for (int i=0; i<no_of_arucos; i++) {
                float curr_dist = sqrt(msg->data[i*no_of_arucos+2]*msg->data[i*no_of_arucos+2] + msg->data[i*no_of_arucos+3]*msg->data[i*no_of_arucos+3]);
                if (msg->data[i*no_of_arucos] == 96) {
                    min_distance    = curr_dist;
                    closest_aruco   = msg->data[i*no_of_arucos];
                    min_index       = i*no_of_arucos;
                    forward_error   = 100+msg->data[i*no_of_arucos+2];
                    lateral_error   = 100+msg->data[i*no_of_arucos+3];
                    std::cout << forward_error << ", " << lateral_error << std::endl;
                    geometry_msgs::Quaternion q;
                    q.w = 0;//heading_error;
                    q.x = forward_error;
                    q.y = lateral_error;
                    error_pub_3d.publish(q);
                }
                else {
                    std::cout << msg->data[i*no_of_arucos] <<std::endl;
                }
            }

        }
        void pixel_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
            int no_of_arucos = msg->data.size()/4;
            float forward_error, lateral_error, heading_error;  
            for (int i=0; i<no_of_arucos; i++) {
                if (msg->data[i*no_of_arucos] == 96) {
                    forward_error = 100+(-1*msg->data[i*no_of_arucos + 2] + 240);
                    lateral_error = 100+(-1*msg->data[i*no_of_arucos + 3] + 320);
                    heading_error = msg->data[i*no_of_arucos + 1]; 
                    std::cout << "forward: "<< forward_error << ", lateral: " << lateral_error << ", yaw: " << heading_error << std::endl;
                    if (yaw_locked==false) {
                        if (sqrt(heading_error*heading_error)<=theta_threshold) {
                            std_srvs::Empty srv;
                            // yaw_lock.call(srv);
                            // yaw_locked = true;
                        }
                    }
                    else {
                        
                    }
                    if (center_called==false) {
                        geometry_msgs::Quaternion q;
                        ROS_INFO("after the center service is not called");
                        q.w = heading_error;
                        q.x = forward_error;
                        q.y = lateral_error;
                        error_pub.publish(q);
                    }       
                }
            }

        }
        void center_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
            float forward_error, lateral_error, heading_error;  
            forward_error = (-1*msg->y + 240);
            lateral_error = (-1*msg->x + 320);
            if (center_called==true) {
                geometry_msgs::Quaternion q;
                ROS_INFO("after the center service is called");
                q.w = heading_error;
                q.x = forward_error;
                q.y = lateral_error;
                error_pub.publish(q);
            }
        } 
        bool emptyServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            center_called = !center_called;
            if (center_called==true) {
                // marked_yaw = yaw_comp_reading;
                ROS_INFO("Service set to true");
            }
            else {
                ROS_INFO("Service set to false");
                // yaw_error = 0;
            }
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "docking_plan");
    ros::NodeHandle                nh;
    Docking24 letsdock(nh);
}