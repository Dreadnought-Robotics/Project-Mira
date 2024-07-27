#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <custom_msgs/commands.h>
#include <std_msgs/Float32.h>
#include "std_srvs/Empty.h"

#define CV_PI 3.1415926535897932384626433832795
#define RESETDEPTH 1100

class Subscriber
{
public:
    bool yaw_locked, center_called, depth_called, autonomy_switch = false, armed = false, reseting = false;
    double depth_error, yaw_error, forward_error, lateral_error, depth_external;
    custom_msgs::commands rov_commands;

    /**
     * @brief Construct a new Subscriber object
     *
     * @param nh NodeHandle to handle ROS communication
     */
    Subscriber(ros::NodeHandle nh)
    {
        error_sub = nh.subscribe("/docking/errors", 1, &Subscriber::dockCallback, this);
        status_sub = nh.subscribe("/docking/status", 1, &Subscriber::statusCallback, this);
        telemetry_sub = nh.subscribe("/master/telemetry", 1, &Subscriber::telemetryCallback, this);
        rov_sub = nh.subscribe("/rov/commands", 1, &Subscriber::rovCallback, this);
    }

private:
    ros::ServiceServer reset_service;
    ros::Subscriber error_sub;
    ros::Subscriber rov_sub;
    ros::Subscriber status_sub;
    ros::Subscriber telemetry_sub;

    /**
     * @brief Callback for dock error messages.
     *
     * @param msg Pointer to the received Quaternion message.
     */
    void dockCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
    {
        yaw_error = msg->w;
        forward_error = msg->x;
        lateral_error = msg->y;
        depth_error = 1040 - depth_external; // Adjust depth error based on external depth
    }

    /**
     * @brief Callback for status messages.
     *
     * @param msg Pointer to the received Vector3 message.
     */
    void statusCallback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        yaw_locked = msg->x;
        center_called = msg->y;
        depth_called = msg->z;
    }

    /**
     * @brief Callback for telemetry messages.
     *
     * @param msg Pointer to the received telemetry message.
     */
    void telemetryCallback(const custom_msgs::telemetry::ConstPtr &msg)
    {
        armed = msg->arm;
        depth_external = msg->external_pressure;
        depth_error = 1040 - depth_external; // Adjust depth error based on external pressure
    }

    /**
     * @brief Callback for ROV command messages.
     *
     * @param msg Received commands message.
     */
    void rovCallback(const custom_msgs::commands msg)
    {
        rov_commands = msg;
    }

    /**
     * @brief Converts a quaternion to a yaw angle in degrees.
     *
     * @param x Quaternion x component.
     * @param y Quaternion y component.
     * @param z Quaternion z component.
     * @param w Quaternion w component.
     * @return Yaw angle in degrees.
     */
    float euler_from_quaternion(double x, double y, double z, double w)
    {
        float yaw;
        double siny_cosp = +2.0 * (w * z + x * y);
        double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan((-1 * siny_cosp) / (-1 * cosy_cosp));
        yaw = yaw * 180 / CV_PI;
        return yaw;
    }
};
