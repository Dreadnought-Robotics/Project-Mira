#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_srvs/Empty.h"


class Subscriber {
    public:
        bool                        service_called = false, depth_service_called = false;
        double                      depth_error, yaw_error, forward_error, lateral_error, pid_depth, pid_yaw;
        double                      angular_velocity_z, yaw_comp_reading, marked_yaw, marked_depth, depth_external;
        Subscriber(ros::NodeHandle nh) {
            yaw_lock                = nh.advertiseService("/yaw/lock", &Subscriber::emptyYawServiceCallback, this);
            depth_lock              = nh.advertiseService("/depth/lock", &Subscriber::emptyDepthServiceCallback, this);
            telemetry_sub           = nh.subscribe("/master/telemetry", 1, &Subscriber::telemetryCallback, this);
            error_sub               = nh.subscribe("/docking/errors", 1, &Subscriber::dockCallback, this);
        }
    private:
        bool emptyYawServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            service_called = !service_called;
            if (service_called==true) {
                marked_yaw = yaw_comp_reading;
                ROS_INFO("Service set to true");
            }
            else {
                ROS_INFO("Service set to false");
                yaw_error = 0;
            }
            return true;
        }
        bool emptyDepthServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
            depth_service_called = !depth_service_called;
            if (depth_service_called==true) {
                marked_depth = depth_external;
                ROS_INFO("Service set to true");
            }
            else {
                ROS_INFO("Service set to false");
                // yaw_error = 0;
            }
            return true;
        }
        ros::ServiceServer          yaw_lock;
        ros::ServiceServer          depth_lock;
        ros::Subscriber             waypoints_sub;
        ros::Subscriber             telemetry_sub;
        ros::Subscriber             error_sub;
        void telemetryCallback(const custom_msgs::telemetry::ConstPtr& msg) {
            depth_error             = 1030 - msg->external_pressure;
            // std::cout << "Printing from subs: " << msg->external_pressure << std::endl;
            depth_external          = msg->external_pressure;
            yaw_comp_reading        = msg->heading;
            if (service_called==true) {
                yaw_error               = marked_yaw - yaw_comp_reading;
                // yaw_error               = 90 - yaw_comp_reading;
            }
            if (depth_service_called==true) {
                depth_error               = marked_depth - depth_external;
                // yaw_error               = 90 - yaw_comp_reading;
            }
        }
        void dockCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
            if (service_called==false) {
                yaw_error               = msg->w;
            }
            forward_error           = msg->x;
            lateral_error           = msg->y;
        }
};


