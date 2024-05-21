#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <custom_msgs/commands.h>
#include <std_msgs/Float32.h>
#include "std_srvs/Empty.h"

#define CV_PI   3.1415926535897932384626433832795

/**
 * @brief Subscriber class for handling ROS subscriptions and service calls.
 */
class Subscriber {
public:
    bool yaw_locked, center_called, depth_called, autonomy_switch = false;
    double depth_error, yaw_error, forward_error, lateral_error, depth_external;
    custom_msgs::commands rov_commands;

    /**
     * @brief Constructor for Subscriber class.
     * 
     * @param nh NodeHandle for ROS communication.
     */
    Subscriber(ros::NodeHandle nh) {
        error_sub = nh.subscribe("/docking/errors", 1, &Subscriber::dockCallback, this);
        status_sub = nh.subscribe("/docking/status", 1, &Subscriber::statusCallback, this);
        telemetry_sub = nh.subscribe("/master/telemetry", 1, &Subscriber::telemetryCallback, this);
        rov_auv = nh.advertiseService("/mira/switch", &Subscriber::switchServiceCallback, this);
    }

private:
    ros::ServiceServer rov_auv;
    ros::Subscriber error_sub;
    ros::Subscriber rov_sub;
    ros::Subscriber status_sub;
    ros::Subscriber telemetry_sub;

    /**
     * @brief Service callback function for switching between autonomy and ROV mode.
     * 
     * @param req Empty service request.
     * @param res Empty service response.
     * @return true If service call succeeds.
     * @return false If service call fails.
     */
    bool switchServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        autonomy_switch = !autonomy_switch;
        if (autonomy_switch) {
            ROS_INFO("AUTONOMOUS MODE");
        } else {
            ROS_INFO("ROV MODE");
        }
        return true;
    }

    /**
     * @brief Callback function for docking errors.
     * 
     * @param msg Quaternion message containing docking errors.
     */
    void dockCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
        yaw_error = msg->w;
        forward_error = msg->x;
        lateral_error = msg->y;
        depth_error = msg->z;
    }

    /**
     * @brief Callback function for docking status.
     * 
     * @param msg Vector3 message containing docking status flags.
     */
    void statusCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        yaw_locked = msg->x;
        center_called = msg->y;
        depth_called = msg->z;
    }

    /**
     * @brief Callback function for receiving telemetry data.
     * 
     * @param msg Telemetry message containing external pressure.
     */
    void telemetryCallback(const custom_msgs::telemetry::ConstPtr& msg) {
        depth_external = msg->external_pressure;
        depth_error = 1050 - depth_external;
    }

    /**
     * @brief Calculate Euler angles from quaternion.
     * 
     * @param x X component of the quaternion.
     * @param y Y component of the quaternion.
     * @param z Z component of the quaternion.
     * @param w W component of the quaternion.
     * @return float Euler angle (yaw) calculated from the quaternion.
     */
    float euler_from_quaternion(double x, double y, double z, double w) {
        float yaw;
        double siny_cosp = +2.0 * (w * z + x * y);
        double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
        yaw = yaw * 180 / CV_PI;
        return yaw;
    }
};
