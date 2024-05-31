#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <ros/ros.h>
#include <custom_msgs/telemetry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>

#define IDTOPLEFT 28
#define IDTOPRIGHT 7
#define IDBOTTOMLEFT 19
#define IDBOTTOMRIGHT 96
#define WAYPOINT_DISTANCE 15
#define MARKER_SIZE 15
#define width 640
#define height 480

/*
   28    80cm      7
   120cm 
   19              96   -------->X
   diagonal = 72.111cm
//640 front                         
// cv::Mat camera_matrix                                   = (cv::Mat_<double>(3, 3) << 664.7437142005466, 0.0, 315.703082526844, 0.0, 669.5841391770296, 252.84811434264267, 0.0, 0.0, 1.0);
// cv::Mat distortion_coefficients                         = (cv::Mat_<double>(1, 5) << -0.4812806594873973, 0.2745181609001952, 0.004042548280670333, -0.006039934872833289, 0.0);
*/

//640 bottom                        
cv::Mat camera_matrix                                    = (cv::Mat_<double>(3, 3) << 655.3957567464429, 0.0, 309.229468237061, 0.0, 658.8623421039053, 223.74985228853447, 0.0, 0.0, 1.0);
cv::Mat distortion_coefficients                          = (cv::Mat_<double>(1, 5) << -0.43734277106914093, 0.20874340537321745, 0.0018488828869952143, 0.0009235703079396009, 0.0);
cv::Ptr<cv::aruco::Dictionary> marker_dict               = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
cv::Ptr<cv::aruco::DetectorParameters> param_markers     = cv::aruco::DetectorParameters::create();
ros::Publisher                                           waypoint_publisher, pixel_publisher, docking_center_publisher;
cv_bridge::CvImagePtr                                    cv_ptr;

class Aruco {
    public:
        int                                     id;
        float                                   roll, pitch, yaw; //pose
        std::vector<cv::Point2f>                marker_corners;
        cv::Vec3d                               rVec, tVec;
        std::vector<float>                      world_coordinates;
        cv::Mat                                 frame;
        int                                     pix_y, pix_x;
        Aruco(int marker_id) {
            id                                  = marker_id;
        }
        void forward() {
            world_coordinates.clear();
            cv::Point2f center(0, 0);
            for (const auto& corner : marker_corners) {
                center += corner;
            }
            center *= 0.25;
            pix_y   = center.y;
            pix_x   = center.x;
            cv::drawFrameAxes(frame, camera_matrix, distortion_coefficients, rVec, tVec, MARKER_SIZE*1.5);
            cv::Mat ret = getCornersInWorldFrame(center, rVec, tVec);
            for (int j=0; j<3; j++) {
                world_coordinates.push_back(ret.at<double>(j,0));
            }
        }
    private:
        void euler_from_quaternion(double x, double y, double z, double w, double &roll, double &pitch, double &yaw) {
            // roll (x-axis rotation)
            double sinr_cosp = +2.0 * (w * x + y * z);
            double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
            roll = atan2(sinr_cosp, cosr_cosp);
            // pitch (y-axis rotation)
            double sinp = +2.0 * (w * y - z * x);
            if (fabs(sinp) >= 1)
                pitch = copysign(M_PI / 2, sinp);
            else
                pitch = asin(sinp);
            // yaw (z-axis rotation)
            double siny_cosp = +2.0 * (w * z + x * y);
            double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
            yaw = atan((-1*siny_cosp)/(-1*cosy_cosp));
        }
        cv::Mat getCornersInWorldFrame(cv::Point2f center, cv::Vec3d rvec, cv::Vec3d tvec){
            cv::Mat translation_vector = cv::Mat::zeros(3, 1, CV_64F);
            translation_vector.at<double>(0) = tvec[0];
            translation_vector.at<double>(1) = tvec[1];
            translation_vector.at<double>(2) = tvec[2];
            cv::Mat inv_cam_matrix = camera_matrix.inv();
            cv::Mat rot_mat;
            cv::Rodrigues(rvec, rot_mat);
            Eigen::Matrix3d eigen_rotation_matrix;
            eigen_rotation_matrix    << rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                                        rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                                        rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2);
            Eigen::Quaterniond quat(eigen_rotation_matrix);
            double transform_rotation_x = quat.x();
            double transform_rotation_y = quat.y();
            double transform_rotation_z = quat.z();
            double transform_rotation_w = quat.w();
            double roll_x, pitch_y, yaw_z;
            euler_from_quaternion(transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w, roll_x, pitch_y, yaw_z);
            roll_x = roll_x * 180 / CV_PI;
            pitch_y = pitch_y * 180 / CV_PI;
            yaw_z = yaw_z * 180 / CV_PI;
            yaw = yaw_z;
            roll = roll_x;
            pitch = pitch_y;
            cv::Mat pixels = cv::Mat::zeros(3,1,CV_64F);
            pixels.at<double>(0) = center.x;
            pixels.at<double>(1) = center.y;
            pixels.at<double>(2) = 1;
            cv::Mat temp = (rot_mat.inv())*(inv_cam_matrix*pixels - translation_vector);
            return temp;
        }
};

Aruco                       left_top(IDTOPLEFT);
Aruco                       left_bottom(IDBOTTOMLEFT);
Aruco                       right_bottom(IDBOTTOMRIGHT);
Aruco                       right_top(IDTOPRIGHT);
float                       depth;
std::vector<Aruco>          aruco_class_vector{left_top, right_top, left_bottom, right_bottom};

void depthCallback(const custom_msgs::telemetry::ConstPtr& msg) {
    depth = msg->external_pressure;
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat  frame= cv_ptr->image;
    // cv::flip(frame_original, frame, -1);
    cv::Mat  gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2f>>   marker_corners;
    std::vector<int>                        marker_IDs;
    std::vector<std::vector<cv::Point2f>>   reject;
    cv::aruco::detectMarkers(gray_frame, marker_dict, marker_corners, marker_IDs, param_markers, reject);

    if (!marker_IDs.empty()) {
        bool found = false;
        std::vector<cv::Vec3d> rVec, tVec;
        cv::aruco::estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, camera_matrix, distortion_coefficients, rVec, tVec);
        std_msgs::Float32MultiArray f, p;
        std::vector<float> aruco;
        for(int i=0; i<marker_IDs.size(); i++) {
            for (int j=0; j<aruco_class_vector.size(); j++) {
                aruco_class_vector[j].frame = frame;
                if (aruco_class_vector[j].id==marker_IDs[i]) {
                    aruco_class_vector[j].marker_corners         = marker_corners[i];
                    aruco_class_vector[j].tVec                   = tVec[i];
                    aruco_class_vector[j].rVec                   = rVec[i];
                    aruco_class_vector[j].forward();
                    f.data.push_back(marker_IDs[i]);
                    f.data.push_back(aruco_class_vector[j].roll);
                    f.data.push_back(aruco_class_vector[j].pitch);
                    f.data.push_back(aruco_class_vector[j].yaw);
                    for (int k=0; k<3; k++) {
                        f.data.push_back(aruco_class_vector[j].world_coordinates[k]);
                    }
                    p.data.push_back(marker_IDs[i]);
                    p.data.push_back(aruco_class_vector[j].yaw);
                    p.data.push_back(aruco_class_vector[j].pix_y);
                    p.data.push_back(aruco_class_vector[j].pix_x);
                    found = true;
                    break;
                }
            }
            if (found==true) {
                ROS_INFO("ARUCO FOUND");
            }
            else {
                ROS_WARN("INVALID ARUCO ID FOUND");
            }
        }
        waypoint_publisher.publish(f);
        pixel_publisher.publish(p);
    }
    // cv::medianBlur(gray_frame, gray_frame, 5);
    // cv::Scalar lower_rgb;
    // cv::Scalar upper_rgb;
    // if (depth<1060) {
    //     lower_rgb={60, 100, 90};
    //     upper_rgb={110, 150, 130};
    //     // lower_rgb={125, 170, 145};
    //     // upper_rgb={160, 190, 170};
    //     // std::cout << depth << std::endl;
    // }
    // else if (depth >1110) {
    //     lower_rgb={60, 60, 60};
    //     upper_rgb={100, 100, 100};  
    // }
    // else {
    //     // lower_rgb={55, 100, 90};
    //     // upper_rgb={100, 150, 130}; 
    //     lower_rgb={120, 130, 160};
    //     upper_rgb={160, 180, 190};  
    // }
    // cv::Mat img_rgb;
    // cv::cvtColor(frame, img_rgb, cv::COLOR_BGR2RGB);
    // cv::Mat mask;
    // cv::inRange(img_rgb, lower_rgb, upper_rgb, mask);
    // std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // int cx, cy;
    // int cy_min = 999, cx_min = 999;
    // for (size_t i = 0; i < contours.size(); i++) {
    //     if (contourArea(contours[i]) > 200) {
    //         cv::Moments M = moments(contours[i]);
    //         if (M.m00 != 0) {
    //             cx = static_cast<int>(M.m10 / M.m00);
    //             cy = static_cast<int>(M.m01 / M.m00);
    //             if (cy<cy_min) {
    //                 cy_min = cy;
    //                 cx_min = cx;
    //             }
    //             drawContours(frame, contours, static_cast<int>(i), cv::Scalar(0, 255, 0), 2);
    //         }
    //     }
    // }
    // cv::circle(frame, cv::Point(cx_min, cy_min), 7, cv::Scalar(0, 0, 255), -1);
    // if (cx_min <640 && cy_min < 640){
    //     geometry_msgs::Vector3 k;
    //     k.x = cx_min;
    //     k.y = cy_min;
    //     docking_center_publisher.publish(k);
    // }
    // int width = frame.cols;
    // int height = frame.rows;
    int center_x = width / 2;
    int center_y = height / 2;
    cv::line(frame, cv::Point(300, center_y), cv::Point(width - 300, center_y), cv::Scalar(0, 255, 0), 2);
    cv::line(frame, cv::Point(center_x, 220), cv::Point(center_x, height - 220), cv::Scalar(0, 255, 0), 2);

    // cv::resizeWindow("Camera Down View", 640, 480);
    // cv::moveWindow("Camera Down View", 1280, 0);
    // cv::imshow("Camera Down View", frame);
    // cv::waitKey(1);
}

//send the feed to a new publisher called /camera_down/image_raw/compressed/aruco 
int main(int argc, char** argv) {
    ros::init(argc, argv, "localizer_using_aruco");
    ros::NodeHandle nh;
    ros::Subscriber image_subscriber    = nh.subscribe("/camera_down/image_raw/compressed", 1, imageCallback);
    ros::Subscriber depth_subscriber    = nh.subscribe("/master/telemetry", 1, depthCallback);
    waypoint_publisher                  = nh.advertise<std_msgs::Float32MultiArray>("/aruco/waypoints", 1);
    // docking_center_publisher            = nh.advertise<geometry_msgs::Vector3>("/docking/center", 1);
    pixel_publisher                     = nh.advertise<std_msgs::Float32MultiArray>("/aruco/pixels", 1);
    ros::spin();
}
