#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <custom_msgs/telemetry.h>
#include <std_msgs/Float32.h>
#define THRESHOLD_CLOCKWISE 60
class Inspect {
    public:
        Inspect(ros::NodeHandle nh) {
            pipe_perception = nh.subscribe<geometry_msgs::Vector3>("/pipeline/errors", 1, &Inspect::perceiveCallback, this);
            error_pub = nh.advertise<geometry_msgs::Quaternion>("/docking/errors", 1);
            telemetry_sub = nh.subscribe<custom_msgs::telemetry>("/master/telemetry", 1, &Inspect::telemetryCallback, this);
            heading_sub = nh.subscribe<std_msgs::Float32>("/mira/heading", 1, &Inspect::headingCallback, this);
            ros::spin();
        }
    private:
        ros::Subscriber pipe_perception;
        ros::Subscriber telemetry_sub;
        ros::Subscriber heading_sub;
        ros::Publisher  error_pub;
        float           depth_reading, heading_reading;
        bool            armed;
        void perceiveCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
            float forward_error, lateral_error, heading_pipeline, depth_error, heading_error;
            // std::cout << "bruh" << std::endl;
            if (armed == true) {
            std::cout << "bruh" << std::endl;

                heading_pipeline = msg->z;
                if (heading_pipeline>THRESHOLD_CLOCKWISE && heading_pipeline<90) {
                    forward_error = (msg->y)*(-1)+240;
                    lateral_error = -50;
                }
                else if (heading_pipeline>90 && heading_pipeline<120) {
                    forward_error = (msg->y)*(-1)+240;
                    lateral_error = 50;
                }
                else {
                    lateral_error = (msg->x)*(-1)+320;
                    forward_error = 50;
                }
                depth_error = 1325 - depth_reading;
                heading_error = -30 - heading_reading;
                if (heading_error < -180)
                    heading_error = heading_error + 360;
                else if (heading_error > 180)
                    heading_error = heading_error - 360;
                geometry_msgs::Quaternion q;
                q.w = heading_error;
                q.x = forward_error;
                q.y = lateral_error;
                q.z = depth_error;
                error_pub.publish(q);
            }
        }
        void telemetryCallback(const custom_msgs::telemetry::ConstPtr &msg)
        {
            depth_reading = msg->external_pressure;
            armed = msg->arm;
        }
        void headingCallback(const std_msgs::Float32::ConstPtr &msg)
        {
            heading_reading = msg->data;
        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv,  "pipeline_inspection");
    ros::NodeHandle nh;
    Inspect letsinspect(nh);
}