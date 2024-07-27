#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>

// Predefined Variables
#define IDTOPLEFT 28
#define IDTOPRIGHT 7
#define IDBOTTOMLEFT 19
#define IDBOTTOMRIGHT 96
#define WAYPOINT_DISTANCE 15
#define MARKER_SIZE 15
#define WIDTH 640
#define HEIGHT 480

// Global Camera Intrinsic Values (640x480)
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 655.3957567464429, 0.0, 309.229468237061, 0.0, 658.8623421039053, 223.74985228853447, 0.0, 0.0, 1.0);
cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << -0.43734277106914093, 0.20874340537321745, 0.0018488828869952143, 0.0009235703079396009, 0.0);

// ArUco marker dictionary and parameters for detection
cv::Ptr<cv::aruco::Dictionary> marker_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
cv::Ptr<cv::aruco::DetectorParameters> param_markers = cv::aruco::DetectorParameters::create();

// ROS Publishers Declarations
ros::Publisher waypoint_publisher, pixel_publisher;
cv_bridge::CvImagePtr cv_ptr;

bool aruco_first_detection = false;

/**
 * @brief Class for handling Aruco marker detection and pose estimation.
 */
class Aruco
{
public:
    int id;
    float roll, pitch, yaw;
    std::vector<cv::Point2f> marker_corners;
    cv::Vec3d rVec, tVec;
    std::vector<float> world_coordinates;
    cv::Mat frame;
    int pix_y, pix_x;

    /**
     * @brief Constructor to initialize Aruco marker with a specific ID.
     * @param marker_id ID of the Aruco marker
     */
    Aruco(int marker_id) : id(marker_id) {}

    /**
     * @brief Calculates the marker center and its world coordinates.
     */
    void forward()
    {
        world_coordinates.clear();
        cv::Point2f center(0, 0);
        for (const auto &corner : marker_corners)
        {
            center += corner;
        }
        center *= 0.25;
        pix_y = center.y;
        pix_x = center.x;
        cv::drawFrameAxes(frame, camera_matrix, distortion_coefficients, rVec, tVec, MARKER_SIZE * 1.5);
        cv::Mat ret = getCornersInWorldFrame(center, rVec, tVec);
        for (int j = 0; j < 3; j++)
        {
            world_coordinates.push_back(ret.at<double>(j, 0));
        }
    }

private:
    /**
     * @brief Converts Euler angles to Quaternion.
     * @param x Quaternion x component
     * @param y Quaternion y component
     * @param z Quaternion z component
     * @param w Quaternion w component
     * @param roll Output roll angle
     * @param pitch Output pitch angle
     * @param yaw Output yaw angle
     */
    void euler_from_quaternion(double x, double y, double z, double w, double &roll, double &pitch, double &yaw)
    {
        // Roll (x-axis rotation)
        double sinr_cosp = +2.0 * (w * x + y * z);
        double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = +2.0 * (w * y - z * x);
        pitch = (fabs(sinp) >= 1) ? copysign(M_PI / 2, sinp) : asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = +2.0 * (w * z + x * y);
        double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
    }

    /**
     * @brief Converts 2D image coordinates to 3D world coordinates.
     * @param center Image coordinates of the marker center
     * @param rvec Rotation vector
     * @param tvec Translation vector
     * @return 3D world coordinates of the marker center
     */
    cv::Mat getCornersInWorldFrame(cv::Point2f center, cv::Vec3d rvec, cv::Vec3d tvec)
    {
        cv::Mat translation_vector = cv::Mat::zeros(3, 1, CV_64F);
        translation_vector.at<double>(0) = tvec[0];
        translation_vector.at<double>(1) = tvec[1];
        translation_vector.at<double>(2) = tvec[2];
        cv::Mat inv_cam_matrix = camera_matrix.inv();
        cv::Mat rot_mat;
        cv::Rodrigues(rvec, rot_mat);
        Eigen::Matrix3d eigen_rotation_matrix;

        eigen_rotation_matrix << rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
            rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
            rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2);

        Eigen::Quaterniond quat(eigen_rotation_matrix);
        double roll_x, pitch_y, yaw_z;
        euler_from_quaternion(quat.x(), quat.y(), quat.z(), quat.w(), roll_x, pitch_y, yaw_z);
        yaw = yaw_z * 180 / CV_PI;
        roll = roll_x * 180 / CV_PI;
        pitch = pitch_y * 180 / CV_PI;
        cv::Mat pixels = cv::Mat::zeros(3, 1, CV_64F);
        pixels.at<double>(0) = center.x;
        pixels.at<double>(1) = center.y;
        pixels.at<double>(2) = 1;
        cv::Mat temp = (rot_mat.inv()) * (inv_cam_matrix * pixels - translation_vector);

        return temp;
    }
};

// Instantiate Aruco marker objects
Aruco left_top(IDTOPLEFT);
Aruco left_bottom(IDBOTTOMLEFT);
Aruco right_bottom(IDBOTTOMRIGHT);
Aruco right_top(IDTOPRIGHT);
float depth;
std::vector<Aruco> aruco_class_vector{left_top, right_top, left_bottom, right_bottom};

/**
 * @brief Callback to update the depth variable from telemetry data.
 * @param msg Depth data message
 */
void depthCallback(const custom_msgs::telemetry::ConstPtr &msg)
{
    depth = msg->external_pressure;
}

/**
 * @brief Callback to process image data, detect ArUco markers, and publish their positions and orientations.
 * @param msg Compressed image message
 */
void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_IDs;
    std::vector<std::vector<cv::Point2f>> reject;
    cv::aruco::detectMarkers(gray_frame, marker_dict, marker_corners, marker_IDs, param_markers, reject);

    if (!marker_IDs.empty())
    {
        bool found = false;
        std::vector<cv::Vec3d> rVec, tVec;
        cv::aruco::estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, camera_matrix, distortion_coefficients, rVec, tVec);
        std_msgs::Float32MultiArray f, p;

        for (size_t i = 0; i < marker_IDs.size(); ++i)
        {
            for (size_t j = 0; j < aruco_class_vector.size(); ++j)
            {
                if (aruco_class_vector[j].id == marker_IDs[i])
                {
                    aruco_class_vector[j].frame = frame;
                    aruco_class_vector[j].marker_corners = marker_corners[i];
                    aruco_class_vector[j].tVec = tVec[i];
                    aruco_class_vector[j].rVec = rVec[i];
                    aruco_class_vector[j].forward();

                    f.data = {marker_IDs[i], aruco_class_vector[j].roll, aruco_class_vector[j].pitch, aruco_class_vector[j].yaw};
                    f.data.insert(f.data.end(), aruco_class_vector[j].world_coordinates.begin(), aruco_class_vector[j].world_coordinates.end());

                    p.data = {marker_IDs[i], aruco_class_vector[j].yaw, static_cast<float>(aruco_class_vector[j].pix_y), static_cast<float>(aruco_class_vector[j].pix_x)};

                    waypoint_publisher.publish(f);
                    pixel_publisher.publish(p);

                    aruco_first_detection = true;
                    ROS_INFO("ARUCO FOUND");
                    found = true;
                    break;
                }
            }

            if (found)
                break;
        }

        if (!found)
        {
            ROS_WARN("INVALID ARUCO ID FOUND");
        }
    }
    else
    {
        std_msgs::Float32MultiArray p;
        if (!aruco_first_detection)
        {
            ROS_WARN("NO ARUCO DETECTED YET");
            p.data = {99, 0, 0, 0};
        }
        pixel_publisher.publish(p);
    }

    int center_x = WIDTH / 2;
    int center_y = HEIGHT / 2;
    cv::line(frame, cv::Point(300, center_y), cv::Point(WIDTH - 300, center_y), cv::Scalar(0, 255, 0), 2);
    cv::line(frame, cv::Point(center_x, 220), cv::Point(center_x, HEIGHT - 220), cv::Scalar(0, 255, 0), 2);
    cv::imshow("Aruco Detection", frame);
}

/**
 * @brief Main function to initialize ROS node, subscribers, and publishers.
 * @param argc Argument count
 * @param argv Argument values
 * @return 0 on successful execution
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer_using_aruco");
    ros::NodeHandle nh;

    // ROS Subscribers
    ros::Subscriber image_subscriber = nh.subscribe("/camera_down/image_raw/compressed", 1, imageCallback);
    ros::Subscriber depth_subscriber = nh.subscribe("/master/telemetry", 1, depthCallback);

    // ROS Publishers
    waypoint_publisher = nh.advertise<std_msgs::Float32MultiArray>("/aruco/waypoints", 1);
    pixel_publisher = nh.advertise<std_msgs::Float32MultiArray>("/aruco/pixels", 1);

    ros::spin();
}
