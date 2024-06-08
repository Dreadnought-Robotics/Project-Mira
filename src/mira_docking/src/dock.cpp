#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <custom_msgs/telemetry.h>
#include <std_msgs/Float32.h>
#include <custom_msgs/commands.h>

// Predefined Variables
#define distance_threshold 15
#define theta_threshold 0.1
#define surface_pressure 1025
#define dock_pressure 1500
#define delta 10

/*
640*480
-1*y, -1*x
-1*y+240, -1*x+320
*/

class Docking24
{
public:
    // Constructor
    Docking24(ros::NodeHandle nh)
    {
        aruco_subscriber = nh.subscribe<std_msgs::Float32MultiArray>("/aruco/pixels", 1, &Docking24::pixel_callback, this);
        center_subscriber = nh.subscribe<geometry_msgs::Vector3>("/docking/center", 1, &Docking24::center_callback, this);
        docking_status = nh.advertise<geometry_msgs::Vector3>("/docking/status", 1);
        telemetry_sub = nh.subscribe<custom_msgs::telemetry>("/master/telemetry", 1, &Docking24::telemetryCallback, this);
        // commands_sub            = nh.subscribe<custom_msgs::commands>("/master/commands", 1, &Docking24::commandsCallback, this);
        heading_sub = nh.subscribe<std_msgs::Float32>("/mira/heading", 1, &Docking24::headingCallback, this);
        error_pub = nh.advertise<geometry_msgs::Quaternion>("/docking/errors", 1);
        yaw_lock = nh.advertiseService("/yaw/lock", &Docking24::emptyYawServiceCallback, this);
        depth_lock = nh.advertiseService("/depth/lock", &Docking24::emptyDepthServiceCallback, this);
        ros::spin();
    }

private:
    // ROS Publishers declarations
    ros::Publisher error_pub;
    ros::Publisher docking_status;

    // ROS Subscribers declarations
    ros::Subscriber aruco_subscriber;
    ros::Subscriber center_subscriber;
    ros::Subscriber heading_sub;
    ros::Subscriber commands_sub;
    ros::Subscriber telemetry_sub;

    // ROS Services declarations
    ros::ServiceServer yaw_lock;
    ros::ServiceServer depth_lock;

    // Control Flags
    bool yaw_locked = false;
    bool center_called = false;
    bool depth_called = false;
    bool armed = false;

    // Control Points Variables
    float heading_mark, heading_reading, depth_mark, depth_reading;
    int marked_aruco, marked_index;

    /* Yaw Service Callback
        Toggles the yaw lock status. If armed and yaw_lock is activated, it sets the heading_mark to the current heading_reading.
    */
    bool emptyYawServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if (armed == true)
        {
            yaw_locked = !yaw_locked;
            if (yaw_locked == true)
            {
                heading_mark = heading_reading;
                std::cout << "marked yaw: " << heading_mark << std::endl;
                ROS_INFO("Yaw Service set to true");
            }
            else
            {
                ROS_INFO("Yaw Service set to false");
                heading_mark = 0;
            }
            return true;
        }
        return false;
    }

    /* Depth Service Callback
        Toggles the depth lock status. If armed and depth_lock is activated, it sets the depth_mark to the current depth_reading.
    */

    bool emptyDepthServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if (armed == true)
        {
            depth_called = !depth_called;
            if (depth_called == true)
            {
                depth_mark = depth_reading;
                ROS_INFO("Depth Service set to true");
            }
            else
            {
                ROS_INFO("Depth Service set to false");
            }
            return true;
        }
        return false;
    }

    /* Aruco Callback using pixel coordinates
        Processes the detected Aruco markers, determines the closest marker, and computes errors for forward, lateral, and heading based on the marker positions.
    */

    void pixel_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {

        int no_of_arucos_detected = msg->data.size() / 4, buffer_X, buffer_Y;
        float forward_error, lateral_error, heading_error;
        float min_distance = 999999;

        for (int i = 0; i < no_of_arucos_detected; i++)
        {
            forward_error = 100 + (-1 * msg->data[i * no_of_arucos_detected + 2] + 240);
            lateral_error = 100 + (-1 * msg->data[i * no_of_arucos_detected + 3] + 320);
            if (min_distance > sqrt(forward_error * forward_error + lateral_error * lateral_error))
            {
                min_distance = sqrt(forward_error * forward_error + lateral_error * lateral_error);
                marked_aruco = msg->data[i * no_of_arucos_detected];
                marked_index = i;
            }
        }

        // Setting x, y delta for center lock
        std::vector<std::vector<int>> buffer{{100, 100}, {100, -100}, {-100, -100}, {-100, 100}};
        int idx = -1;

        if (marked_aruco == 96)
            idx = 0;
        else if (marked_aruco == 19)
            idx = 1;
        else if (marked_aruco == 28)
            idx = 2;
        else if (marked_aruco == 7)
            idx = 3;

        if (idx != -1)
        {
            buffer_X = buffer[idx][0];
            buffer_Y = buffer[idx][1];
        }

        if (msg->data[marked_index * no_of_arucos_detected] == marked_aruco && center_called == false)
        {
            // ROS_INFO("Marked Aruco: %d, X: %f, Y: %f", marked_aruco, -1*msg->data[marked_index*no_of_arucos_detected + 2] + 240, -1*msg->data[marked_index*no_of_arucos_detected + 3] + 320);
            forward_error = buffer_X + (-1 * msg->data[marked_index * no_of_arucos_detected + 2] + 240);
            lateral_error = buffer_Y + (-1 * msg->data[marked_index * no_of_arucos_detected + 3] + 320);

            if (yaw_locked == false && armed == true)
            {
                heading_error = msg->data[marked_index * no_of_arucos_detected + 1];

                if (sqrt(heading_error * heading_error) <= theta_threshold)
                {
                    yaw_locked = true;
                    heading_mark = heading_reading;
                    std::cout << "Marked Yaw: " << heading_mark << std::endl;
                    ROS_INFO("Yaw Lock is Enabled");
                }
            }

            else
            {
                // Comment the below if condition to disable center lock
                if (sqrt(forward_error * forward_error) <= distance_threshold && sqrt(lateral_error * lateral_error) <= distance_threshold)
                {
                    center_called = true;
                    ROS_INFO("Center Lock is Enabled");
                }
                heading_error = heading_mark - heading_reading;
            }

            if (heading_error < -180)
                heading_error = heading_error + 360;
            else if (heading_error > 180)
                heading_error = heading_error - 360;
        }

        geometry_msgs::Quaternion q;
        q.w = heading_error;
        q.x = forward_error;
        q.y = lateral_error;
        q.z = 1025 - depth_reading;

        error_pub.publish(q);
    }

    /* Center Callback
        Processes the docking center position, computes forward, lateral, heading, and depth errors, and publishes them.
    */
    void center_callback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        float forward_error, lateral_error, heading_error, depth_error;

        if (center_called == true)
        {
            forward_error = (-1 * msg->y + 240);
            lateral_error = (-1 * msg->x + 320);
            heading_error = heading_mark - heading_reading;

            if (heading_error < -180)
                heading_error = heading_error + 360;
            else if (heading_error > 180)
                heading_error = heading_error - 360;

            if (depth_called == false)
            {
                if (sqrt(forward_error * forward_error) <= distance_threshold && sqrt(lateral_error * lateral_error) <= distance_threshold)
                {
                    depth_called = true;
                    ROS_INFO("Depth Lock is Enabled");
                }
                depth_error = 1025 - depth_reading;
            }
            else
            {
                if (depth_reading < surface_pressure)
                {
                    depth_error = surface_pressure - depth_reading;
                }
                else if (depth_reading > 1025 && depth_reading <= 1035)
                {
                    depth_error = 1035 - depth_reading;
                }
                else if (depth_reading > 1035 && depth_reading <= 1045)
                {
                    depth_error = 1045 - depth_reading;
                }
                else if (depth_reading > 1045 && depth_reading <= 1055)
                {
                    depth_error = 1055 - depth_reading;
                }
                else if (depth_reading > 1055 && depth_reading <= 1065)
                {
                    depth_error = 1065 - depth_reading;
                }
                else if (depth_reading > 1065 && depth_reading <= 1075)
                {
                    depth_error = 1075 - depth_reading;
                }
                else if (depth_reading > 1075 && depth_reading <= 1085)
                {
                    depth_error = 1085 - depth_reading;
                }
                else if (depth_reading > 1085 && depth_reading <= 1095)
                {
                    depth_error = 1095 - depth_reading;
                }
                else if (depth_reading > 1095 && depth_reading <= 1105)
                {
                    depth_error = 1105 - depth_reading;
                }
                else if (depth_reading > 1105 && depth_reading <= 1115)
                {
                    depth_error = 1115 - depth_reading;
                }
                else if (depth_reading > 1115 && depth_reading <= 1125)
                {
                    depth_error = 1125 - depth_reading;
                }
                else if (depth_reading > 1125 && depth_reading <= 1135)
                {
                    depth_error = 1135 - depth_reading;
                }
                else if (depth_reading > 1135 && depth_reading <= 1145)
                {
                    depth_error = 1145 - depth_reading;
                }
                else if (depth_reading > 1145 && depth_reading <= 1155)
                {
                    depth_error = 1155 - depth_reading;
                }
                else if (depth_reading > 1155 && depth_reading <= 1165)
                {
                    depth_error = 1165 - depth_reading;
                }
                else if (depth_reading > 1165 && depth_reading <= 1175)
                {
                    depth_error = 1175 - depth_reading;
                }
                else if (depth_reading > 1175 && depth_reading <= 1185)
                {
                    depth_error = 1185 - depth_reading;
                }
                else if (depth_reading > 1185 && depth_reading <= 1195)
                {
                    depth_error = 1195 - depth_reading;
                }
                else if (depth_reading > 1195 && depth_reading <= 1205)
                {
                    depth_error = 1205 - depth_reading;
                }
                else if (depth_reading > 1205 && depth_reading <= 1215)
                {
                    depth_error = 1215 - depth_reading;
                }
                else if (depth_reading > 1215 && depth_reading <= 1225)
                {
                    depth_error = 1225 - depth_reading;
                }
                else if (depth_reading > 1225 && depth_reading <= 1235)
                {
                    depth_error = 1235 - depth_reading;
                }
                else if (depth_reading > 1235 && depth_reading <= 1245)
                {
                    depth_error = 1245 - depth_reading;
                }
                else if (depth_reading > 1245 && depth_reading <= 1250)
                {
                    depth_error = 1250 - depth_reading;
                }
                else
                {
                    depth_error = dock_pressure - depth_reading;
                }
            }

            geometry_msgs::Quaternion q;
            q.w = heading_error;
            q.x = forward_error;
            q.y = lateral_error;
            q.z = depth_error;

            error_pub.publish(q);
        }
    }

    /* Telemetry Callback
        Updates the depth reading and armed status. Resets control flags if the system is disarmed.
    */
    void telemetryCallback(const custom_msgs::telemetry::ConstPtr &msg)
    {
        depth_reading = msg->external_pressure;
        armed = msg->arm;
        if (armed == false)
        {
            center_called = false;
            yaw_locked = false;
            depth_called = false;
            // ROS_INFO("Depth Lock is Disabled");
            // ROS_INFO("Center Lock is Disabled");
            // ROS_INFO("Yaw Lock is Disabled");
        }
        geometry_msgs::Quaternion p;

        if (yaw_locked == false)
            p.x = 0;
        else
            p.x = 1;

        if (center_called == false)
            p.y = 0;
        else
            p.y = 1;

        if (depth_called == false)
            p.z = 0;
        else
            p.z = 1;

        docking_status.publish(p);
    }

    // void commandsCallback(const custom_msgs::commands::ConstPtr& msg) {

    // }

    /* Heading Callback
        Updates the current heading reading.
    */
    void headingCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        heading_reading = msg->data;
        // std::cout << "heafi: " << heading_reading<<std::endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "docking_plan");
    ros::NodeHandle nh;
    Docking24 letsdock(nh);
}