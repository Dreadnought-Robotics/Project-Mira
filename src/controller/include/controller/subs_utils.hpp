#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

class Subscriber {
    public:
        double          thrust_error, yaw_error, forward_error;
        double          angular_velocity_z;
        Subscriber(ros::NodeHandle nh) {
            telemetry_sub           = nh.subscribe("/aruco/coordinates", 1, &Subscriber::telemetryCallback, this);
        }
    private:
        ros::Subscriber             coordinates_sub;
        ros::Subscriber             telemetry_sub;
        void telemetryCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
            // if ((msg->y >= 240 && msg->x >= 240) || (msg->y <= 240 && msg->x <= 240)) {
                yaw_error               = std::atan2(((msg->y)), ((msg->x)))*57.2958;
            // }
            // else {
                // yaw_error               = std::atan2(((msg->x * (-1)) + 240), ((msg->y * (-1)) + 240))*57.2958;
            // }
        }
};


