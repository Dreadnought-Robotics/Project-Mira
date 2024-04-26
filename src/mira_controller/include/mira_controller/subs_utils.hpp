#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>

class Subscriber {
    public:
        double                      depth_error, yaw_error, forward_error, lateral_error, pid_depth, pid_yaw;
        double                      angular_velocity_z;
        Subscriber(ros::NodeHandle nh) {
            telemetry_sub           = nh.subscribe("/master/telemetry", 1, &Subscriber::telemetryCallback, this);
            error_sub               = nh.subscribe("/docking/errors", 1, &Subscriber::dockCallback, this);
        }
    private:
        ros::Subscriber             waypoints_sub;
        ros::Subscriber             telemetry_sub;
        ros::Subscriber             error_sub;
        void telemetryCallback(const custom_msgs::telemetry::ConstPtr& msg) {
            depth_error             = 0;
        }
        void dockCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
            yaw_error               = msg->w;
            forward_error           = msg->x;
            lateral_error           = msg->y;
        }
};


