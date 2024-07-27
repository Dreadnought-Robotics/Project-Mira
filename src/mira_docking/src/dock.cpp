#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <custom_msgs/telemetry.h>
#include <std_msgs/Float32.h>
#include <custom_msgs/commands.h>

// Predefined constants
#define DISTANCE_THRESHOLD 15
#define THETA_THRESHOLD 0.1
#define SURFACE_PRESSURE 1100
#define DOCK_PRESSURE 1234 /
#define DELTA 10

// Class for handling docking operations
class Docking24
{
public:
    /**
     * @brief Constructor that initializes ROS publishers, subscribers, and services.
     *
     * @param nh NodeHandle for setting up ROS communication.
     */
    Docking24(ros::NodeHandle nh)
    {
        aruco_subscriber = nh.subscribe<std_msgs::Float32MultiArray>("/aruco/pixels", 1, &Docking24::pixel_callback, this);
        center_subscriber = nh.subscribe<geometry_msgs::Vector3>("/docking/center", 1, &Docking24::center_callback, this);
        docking_status = nh.advertise<geometry_msgs::Vector3>("/docking/status", 1);
        telemetry_sub = nh.subscribe<custom_msgs::telemetry>("/master/telemetry", 1, &Docking24::telemetryCallback, this);
        heading_sub = nh.subscribe<std_msgs::Float32>("/mira/heading", 1, &Docking24::headingCallback, this);
        error_pub = nh.advertise<geometry_msgs::Quaternion>("/docking/errors", 1);
        yaw_lock = nh.advertiseService("/yaw/lock", &Docking24::emptyYawServiceCallback, this);
        depth_lock = nh.advertiseService("/depth/lock", &Docking24::emptyDepthServiceCallback, this);

        ros::spin(); // Enter ROS event loop
    }

private:
    // ROS Publishers
    ros::Publisher error_pub;
    ros::Publisher docking_status;

    // ROS Subscribers
    ros::Subscriber aruco_subscriber;
    ros::Subscriber center_subscriber;
    ros::Subscriber heading_sub;
    ros::Subscriber telemetry_sub;

    // ROS Services
    ros::ServiceServer yaw_lock;
    ros::ServiceServer depth_lock;

    // Control Flags
    bool yaw_locked = false;
    bool center_called = false;
    bool depth_called = false;
    bool armed = false;

    // Control Points Variables
    float heading_mark = 0;
    float heading_reading = 0;
    float depth_mark = 0;
    float depth_reading = 0;
    int marked_aruco = 0;
    int marked_index = 0;

    /**
     * @brief Callback for the yaw lock service.
     * Toggles the yaw lock status and sets the heading mark if armed.
     *
     * @param req Request message (not used)
     * @param res Response message (not used)
     * @return True if the service call was successful, false otherwise
     */
    bool emptyYawServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if (armed)
        {
            yaw_locked = !yaw_locked;
            if (yaw_locked)
            {
                heading_mark = heading_reading;
                ROS_INFO("Yaw locked at: %f", heading_mark);
            }
            else
            {
                heading_mark = 0;
                ROS_INFO("Yaw unlock");
            }
            return true;
        }
        return false;
    }

    /**
     * @brief Callback for the depth lock service.
     * Toggles the depth lock status and sets the depth mark if armed.
     *
     * @param req Request message (not used)
     * @param res Response message (not used)
     * @return True if the service call was successful, false otherwise
     */
    bool emptyDepthServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if (armed)
        {
            depth_called = !depth_called;
            if (depth_called)
            {
                depth_mark = depth_reading;
                ROS_INFO("Depth lock enabled");
            }
            else
            {
                ROS_INFO("Depth lock disabled");
            }
            return true;
        }
        return false;
    }

    /**
     * @brief Callback for processing Aruco marker pixel data.
     * Computes errors based on marker positions and updates the docking state.
     *
     * @param msg Aruco marker pixel data
     */
    void pixel_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        int num_arucos_detected = msg->data.size() / 4;
        float forward_error = 0, lateral_error = 0, heading_error = 0;
        float min_distance = std::numeric_limits<float>::max();

        if (num_arucos_detected == 1 && msg->data[0] == 99)
        {
            forward_error = 75;
        }
        else
        {
            for (int i = 0; i < num_arucos_detected; ++i)
            {
                float x = -1 * msg->data[i * 4 + 2] + 240;
                float y = -1 * msg->data[i * 4 + 3] + 320;
                float distance = sqrt(x * x + y * y);

                if (distance < min_distance)
                {
                    min_distance = distance;
                    marked_aruco = msg->data[i * 4];
                    marked_index = i;
                }
            }

            // Determine buffer coordinates based on Aruco marker ID
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

            int buffer_X = 0, buffer_Y = 0;
            if (idx != -1)
            {
                buffer_X = buffer[idx][0];
                buffer_Y = buffer[idx][1];
            }

            if (msg->data[marked_index * 4] == marked_aruco && !center_called && armed)
            {
                forward_error = -1 * msg->data[marked_index * 4 + 2] + 240;
                lateral_error = -1 * msg->data[marked_index * 4 + 3] + 320;

                if (!yaw_locked)
                {
                    heading_error = msg->data[marked_index * 4 + 1];
                    if (fabs(heading_error) <= THETA_THRESHOLD)
                    {
                        yaw_locked = true;
                        heading_mark = heading_reading;
                        ROS_INFO("Yaw lock enabled at: %f", heading_mark);
                    }
                }
                else
                {
                    if (sqrt(forward_error * forward_error) <= DISTANCE_THRESHOLD &&
                        sqrt(lateral_error * lateral_error) <= DISTANCE_THRESHOLD && !center_called)
                    {
                        center_called = true;
                        ROS_INFO("Center lock enabled");
                    }
                    heading_error = heading_mark - heading_reading;
                }

                if (heading_error < -180)
                    heading_error += 360;
                else if (heading_error > 180)
                    heading_error -= 360;
            }
        }

        // Publish error data
        geometry_msgs::Quaternion q;
        q.w = heading_error;
        q.x = forward_error;
        q.y = lateral_error;
        q.z = SURFACE_PRESSURE - depth_reading;

        error_pub.publish(q);
    }

    /**
     * @brief Callback for processing the docking center position.
     * Computes errors for forward, lateral, heading, and depth, and publishes them.
     *
     * @param msg Docking center position
     */
    void center_callback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        float forward_error = 0, lateral_error = 0, heading_error = 0, depth_error = 0;

        if (center_called && armed)
        {
            forward_error = -1 * msg->y + 240;
            lateral_error = -1 * msg->x + 320;
            heading_error = heading_mark - heading_reading;

            if (heading_error < -180)
                heading_error += 360;
            else if (heading_error > 180)
                heading_error -= 360;

            if (!depth_called)
            {
                if (sqrt(forward_error * forward_error) <= DISTANCE_THRESHOLD &&
                    sqrt(lateral_error * lateral_error) <= DISTANCE_THRESHOLD)
                {
                    depth_called = true;
                    ROS_INFO("Depth lock enabled");
                }
                depth_error = SURFACE_PRESSURE - depth_reading;
            }
            else
            {
                if (depth_reading < SURFACE_PRESSURE)
                {
                    depth_error = SURFACE_PRESSURE - depth_reading;
                }
                else if (depth_reading <= 1115)
                {
                    depth_error = 1115 - depth_reading;
                }
                else if (depth_reading <= 1130)
                {
                    depth_error = 1130 - depth_reading;
                }
                else if (depth_reading <= 1145)
                {
                    depth_error = 1145 - depth_reading;
                }
                else if (depth_reading <= 1160)
                {
                    depth_error = 1160 - depth_reading;
                }
                else if (depth_reading <= 1175)
                {
                    depth_error = 1175 - depth_reading;
                }
                else if (depth_reading <= 1190)
                {
                    depth_error = 1190 - depth_reading;
                }
                else if (depth_reading <= 1220)
                {
                    depth_error = 1220 - depth_reading;
                }
                else
                {
                    depth_error = DOCK_PRESSURE - depth_reading;
                }
            }

            // Publish error data
            geometry_msgs::Quaternion q;
            q.w = heading_error;
            q.x = forward_error;
            q.y = lateral_error;
            q.z = depth_error;

            error_pub.publish(q);
        }
    }

    /**
     * @brief Callback for updating the depth reading and armed status from telemetry data.
     * Resets control flags if the system is disarmed.
     *
     * @param msg Telemetry data
     */
    void telemetryCallback(const custom_msgs::telemetry::ConstPtr &msg)
    {
        depth_reading = msg->external_pressure;
        armed = msg->arm;

        if (!armed)
        {
            center_called = false;
            yaw_locked = false;
            depth_called = false;
            ROS_INFO("System disarmed. All locks disabled.");
        }

        // Publish docking status
        geometry_msgs::Quaternion p;
        p.x = yaw_locked ? 1 : 0;
        p.y = center_called ? 1 : 0;
        p.z = depth_called ? 1 : 0;

        docking_status.publish(p);
    }

    /**
     * @brief Callback for updating the current heading reading.
     *
     * @param msg Heading data
     */
    void headingCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        heading_reading = msg->data;
    }
};

/**
 * @brief Main function for initializing the ROS node and the Docking24 class.
 *
 * @param argc Argument count
 * @param argv Argument values
 * @return 0 on successful execution
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "docking_plan");
    ros::NodeHandle nh;
    Docking24 letsdock(nh);

    return 0;
}
