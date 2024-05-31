#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <custom_msgs/commands.h>
#include <std_msgs/Float32.h>
#include "std_srvs/Empty.h"
#define CV_PI   3.1415926535897932384626433832795

class Subscriber {
    public:
        bool                        yaw_locked, center_called, depth_called, autonomy_switch=false, armed = false;
        double                      depth_error, yaw_error, forward_error, lateral_error, depth_external;
        custom_msgs::commands       rov_commands;
        Subscriber(ros::NodeHandle nh) {
            error_sub               = nh.subscribe("/docking/errors", 1, &Subscriber::dockCallback, this);
            status_sub              = nh.subscribe("/docking/status", 1, &Subscriber::statusCallback, this);
            telemetry_sub           = nh.subscribe("/master/telemetry", 1, &Subscriber::telemetryCallback, this);
            rov_sub                 = nh.subscribe("/rov/commands", 1, &Subscriber::rovCallback, this);
            // rov_auv                 = nh.advertiseService("/mira/switch", &Subscriber::switchServiceCallback, this);
        }
    private:
        ros::ServiceServer          rov_auv;
        ros::Subscriber             error_sub;
        ros::Subscriber             rov_sub;
        ros::Subscriber             status_sub;
        ros::Subscriber             telemetry_sub;
        // bool switchServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        //     autonomy_switch = !autonomy_switch;
        //     if (autonomy_switch==true) {
        //         ROS_INFO("AUTONOMOUS MODE");     
        //     }
        //     else {
        //         ROS_INFO("ROV MODE");
        //     }
        //     return true;
        // }
        void dockCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
            yaw_error               = msg->w;
            forward_error           = msg->x;
            lateral_error           = msg->y;
            depth_error             = msg->z;
            // yaw_error               = 0;
            // forward_error           = 0;
            // lateral_error           = 0;
            depth_error             = 1025 - depth_external;
            // std::cout << depth_error << std::endl;
        }
        void statusCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
            yaw_locked              = msg->x;
            center_called           = msg->y;
            depth_called            = msg->z;
        }
        void telemetryCallback(const custom_msgs::telemetry::ConstPtr& msg) {
            armed                   = msg->arm;
            depth_external          = msg->external_pressure;
            depth_error             = 1025 - depth_external;
        }
        void rovCallback(const custom_msgs::commands msg) {
            rov_commands            = msg;
        }
        float euler_from_quaternion(double x, double y, double z, double w) {
            float yaw;
            double siny_cosp = +2.0 * (w * z + x * y);
            double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
            yaw = atan((-1*siny_cosp)/(-1*cosy_cosp));
            yaw = yaw * 180 / CV_PI;
            return yaw;
        }

};


