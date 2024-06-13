#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <numeric>  // Add this line for std::accumulate
// Global variables for color range
cv::Scalar lower_rgb(0, 0, 0);
cv::Scalar upper_rgb(180, 180, 150);

// Function to apply CLAHE
cv::Mat CLAHE(const cv::Mat &image) {
    cv::Mat gray_image, clahe_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = image;
    }
    auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray_image, clahe_image);
    if (image.channels() == 3) {
        cv::cvtColor(clahe_image, clahe_image, cv::COLOR_GRAY2BGR);
    }
    return clahe_image;
}

// Function to apply white balance
cv::Mat white_balance(const cv::Mat &image) {
    cv::Mat lab_image, balanced_lab_image, balanced_image;
    cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);
    auto clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(lab_planes[0], lab_planes[0]);
    cv::merge(lab_planes, balanced_lab_image);
    cv::cvtColor(balanced_lab_image, balanced_image, cv::COLOR_Lab2BGR);
    return balanced_image;
}

// Function to increase contrast
cv::Mat Contrast_Up(const cv::Mat &image) {
    cv::Mat contrasted_image;
    image.convertTo(contrasted_image, -1, 0.5, 0);
    return contrasted_image;
}

// Function to decrease contrast
cv::Mat Contrast_Down(const cv::Mat &image) {
    cv::Mat contrasted_image;
    image.convertTo(contrasted_image, -1, 0.2, 0);
    return contrasted_image;
}

// Function to increase brightness
cv::Mat Brightness_Up(const cv::Mat &image) {
    cv::Mat brightened_image;
    image.convertTo(brightened_image, -1, 2, 150);
    return brightened_image;
}

// Function to decrease brightness
cv::Mat Brightness_Down(const cv::Mat &image) {
    cv::Mat darkened_image;
    image.convertTo(darkened_image, -1, 1.0, 10);
    return darkened_image;
}

// Function to normalize angle
double Angle_Normalize(double angle) {
    if (angle > 90) {
        angle = angle - 180;
    }
    return angle;
}

// Function to apply masking
cv::Mat Masking(const cv::Mat &image) {
    cv::Mat hsv_image, mask, new_img;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, lower_rgb, upper_rgb, mask);
    cv::bitwise_and(image, image, new_img, mask);
    cv::GaussianBlur(new_img, new_img, cv::Size(55, 55), 0);

    cv::Mat kernel = (cv::Mat_<float>(3, 3) << -1, -1, -1, -1, 99, -1, -1, -1, -1);
    cv::filter2D(new_img, new_img, -1, kernel);

    cv::Mat temp_img;
    new_img.copyTo(temp_img);
    cv::bilateralFilter(temp_img, new_img, 9, 1000, 1000);  // Ensure src and dst are different
    return new_img;
}


// Function to make Hough lines
std::pair<cv::Mat, double> Make_Hough_Lines(const cv::Mat &image) {
    cv::Mat gray_image, edges;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_image, gray_image, cv::Size(55, 55), 0);
    cv::Mat sobel_x, sobel_y;
    cv::Sobel(gray_image, sobel_x, CV_64F, 1, 0, 3);
    cv::Sobel(gray_image, sobel_y, CV_64F, 0, 1, 3);
    cv::Mat gradient_magnitude;
    cv::sqrt(sobel_x.mul(sobel_x) + sobel_y.mul(sobel_y), gradient_magnitude);
    cv::threshold(gradient_magnitude, edges, 50, 255, cv::THRESH_BINARY);
    edges.convertTo(edges, CV_8U);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 1, CV_PI / 180, 100);

    double average_angle = 0.0;
    if (!lines.empty()) {
        std::vector<double> angles;
        for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0], theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
            cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 2);
            angles.push_back(theta);
        }
        average_angle = std::accumulate(angles.begin(), angles.end(), 0.0) / angles.size();
        average_angle = average_angle * 180.0 / CV_PI; // Convert to degrees
    }
    return std::make_pair(image, average_angle);
}

// Callback function
void main_Callback(const sensor_msgs::CompressedImageConstPtr &msg, const ros::Publisher &pub) {
    int cx = 0, cy = 0;
    cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    int height = frame.rows, width = frame.cols;
    int crop_width = width / 2, crop_height = height / 2;
    int crop_x = width - crop_width, crop_y = 0;
    int part_height = height / 3;

    cv::Mat img_rgb;
    cv::cvtColor(frame, img_rgb, cv::COLOR_BGR2RGB);
cv::Scalar lower_rgb(150, 150, 150);
cv::Scalar upper_rgb(255, 255, 255);
    cv::Mat mask;
    cv::inRange(img_rgb, lower_rgb, upper_rgb, mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto &cnt : contours) {
        if (cv::contourArea(cnt) > 200) {
            cv::Moments M = cv::moments(cnt);
            if (M.m00 != 0) {
                cx = static_cast<int>(M.m10 / M.m00);
                cy = static_cast<int>(M.m01 / M.m00);
            } else {
                cx = 0;
                cy = 0;
            }
        }
    }

    cv::Mat part1 = frame(cv::Rect(0, 0, width, part_height));
    cv::Mat part2 = frame(cv::Rect(0, part_height, width, part_height));
    // cv::Mat part3 = frame(cv::Rect(0, 2 * part_height, width, part_height));
    cv::imshow("mask", mask);
    auto [n_part1, avg1] = Make_Hough_Lines(Masking(part1));
    auto [n_part2, avg2] = Make_Hough_Lines(Masking(part2));
    // auto [n_part3, avg3] = Make_Hough_Lines(Masking(part3));

    cv::imshow("orig Part3", frame);
    cv::waitKey(1);

    if (avg1 && avg2) {
        avg1 = Angle_Normalize(avg1);
        avg2 = Angle_Normalize(avg2);

        double Angle_diff = abs(avg1 - avg2);
        ROS_INFO("Angle Difference: %f", Angle_diff);

        geometry_msgs::Vector3 m;
        m.x = cx;
        m.y = cy;
        m.z = Angle_diff;
        pub.publish(m);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "angle_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/pipeline/errors", 10);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera_down/image_enhanced", 10, 
    boost::bind(main_Callback, _1, pub));
    ros::spin();
    return 0;
}
