#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_srvs/Empty.h"


class Subscriber {
    public:
        bool                        service_called = false;
        double                      depth_error, yaw_error, forward_error, lateral_error, pid_depth, pid_yaw;
        double                      angular_velocity_z, yaw_comp_reading, marked_yaw;
        Subscriber(ros::NodeHandle nh) {
            yaw_lock                = nh.advertiseService("/yaw/lock", &Subscriber::emptyServiceCallback, this);
            telemetry_sub           = nh.subscribe("/master/telemetry", 1, &Subscriber::telemetryCallback, this);
            error_sub               = nh.subscribe("/docking/errors", 1, &Subscriber::dockCallback, this);
        }
    private:
        bool emptyServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
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
        ros::ServiceServer          yaw_lock;
        ros::Subscriber             waypoints_sub;
        ros::Subscriber             telemetry_sub;
        ros::Subscriber             error_sub;
        void telemetryCallback(const custom_msgs::telemetry::ConstPtr& msg) {
            depth_error             = 0;
            yaw_comp_reading        = msg->heading;
            if (service_called==true) {
                yaw_error               = marked_yaw - yaw_comp_reading;
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


