#include <ros/ros.h>
#include <mira_controller/subs_utils.hpp>
#include <mira_controller/tuning_utils.hpp>
#include <mira_controller/control_utils.hpp>
#include <custom_msgs/commands.h>
#include <std_msgs/Char.h>

//   +ve   x  -ve (CCW)
//   (CW)  |   
//   y -- bot 
//   +ve

Control yaw, depth, forward, lateral;
std::vector<double> pid_constants{2,0,0};
#define threshold 8 //degrees

custom_msgs::commands cmd_pwm;
bool tuning_mode = false;
char tuning_axis = '\0';
void keys_callback(const std_msgs::Char::ConstPtr& msg) {
    
char key = msg->data;
    if (key == 'a') {
        cmd_pwm.arm = true;
        std::cout<<"Mira Armed"<<std::endl;
    } 
    else if (key == 'z') {
        cmd_pwm.arm = false;
        std::cout<<"Mira Disarmed"<<std::endl;
    }
    else if (key == 'q') {
        tuning_mode = false;
        tuning_axis = '\0';
        std::cout<<"--------PID values for each axis--------"<<std::endl;
        std::cout<<"Depth PID: "<<depth.kp<<","<<depth.ki<<","<<depth.kd<<std::endl;
        std::cout<<"Forward PID: "<<forward.kp<<","<<forward.ki<<","<<forward.kd<<std::endl;
        std::cout<<"Lateral PID: "<<lateral.kp<<","<<lateral.ki<<","<<lateral.kd<<std::endl;
        std::cout<<"Yaw PID: "<<yaw.kp<<","<<yaw.ki<<","<<yaw.kd<<std::endl;        
    } 
    else if (key == 'd') {
        tuning_mode = true;
        tuning_axis = 'd';
        std::cout<<"Tuning depth PID"<<std::endl;
    } 
    else if (key == 'y') {
        tuning_mode = true;
        tuning_axis = 'y';
        std::cout<<"Tuning yaw PID"<<std::endl;
    } 
    else if (key == 'f') {
        tuning_mode = true;
        tuning_axis = 'f';
        std::cout<<"Tuning forward PID"<<std::endl;
    }
    else if (key == 'l') {
        tuning_mode = true;
        tuning_axis = 'l';
        std::cout<<"Tuning lateral PID"<<std::endl;
    } 
    else if (tuning_mode) {
        float val = key;
        switch (tuning_axis) {
            case 'd':
                if (key == '1') depth.kp += val;
                else if (key == '2') depth.ki += val;
                else if (key == '3') depth.kd += val;
                std::cout<<"Current depth PID values: "<<depth.kp<<" "<<depth.ki<<" "<<depth.kd<<std::endl;
                break;
            case 'y':
                if (key == '1') yaw.kp += val;
                else if (key == '2') yaw.ki += val;
                else if (key == '3') yaw.kd += val;
                std::cout<<"Current yaw PID values: "<<yaw.kp<<" "<<yaw.ki<<" "<<yaw.kd<<std::endl;
                break;
            case 'f':
                if (key == '1') forward.kp += val;
                else if (key == '2') forward.ki += val;
                else if (key == '3') forward.kd += val;
                std::cout<<"Current forward PID values: "<<forward.kp<<" "<<forward.ki<<" "<<forward.kd<<std::endl;
                break;
            case 'l':
                if (key == '1') lateral.kp += val;
                else if (key == '2') lateral.ki += val;
                else if (key == '3') lateral.kd += val;
                std::cout<<"Current lateral PID values: "<<lateral.kp<<" "<<lateral.ki<<" "<<lateral.kd<<std::endl;
                break;
        }
    } 
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "auv_controller");
    ros::NodeHandle nh;
    ros::Publisher  pwm_publisher         = nh.advertise<custom_msgs::commands>("/master/commands", 1);
    ros::Publisher  error_publisher       = nh.advertise<custom_msgs::commands>("/depth/error", 1);
    ros::Subscriber keys_subscriber       = nh.subscribe("keys", 1, keys_callback);

    Subscriber          subs(nh);
    GradientDescent     descent(0.01);
    // yaw.kp      = 3.69;
    // yaw.ki      = 0;
    // yaw.kd      = 0;
    depth.kp    = pid_constants[0];
    depth.ki    = pid_constants[1];
    depth.kd    = pid_constants[2];
    bool arm    = true;
    double init_time = ros::Time::now().toSec();
    double prev_time = ros::Time::now().toSec();
    cmd_pwm.arm = true;
    char c;
    cmd_pwm.mode="STABILIZE";
    float _h = 0.000001;
    double time_interval = 10.0;
    std::vector<double> cost_h{0,0,0};
    double cost;
    float dt = 0.05;
    ros::Rate loop_rate = dt;
    std::vector<double> cost_values;
    int iterations = 0;
    bool increasing_h = false, increasing_h_kp = false, increasing_h_ki = false, increasing_h_kd = false;
    while (ros::ok()) {
        char key;

        double time_now      = ros::Time::now().toSec();
        if (!forward.error_vector.empty()) {
            auto it                     = forward.error_vector.end();
            if (*it - subs.forward_error > threshold) {
                int num_elements        = std::min(static_cast<int>(forward.error_vector.size()), 10);
                auto max_element_iter   = std::max_element(forward.error_vector.end() - num_elements, forward.error_vector.end());
                subs.forward_error      = *max_element_iter;
            }
            else if(*it - subs.forward_error < -1*threshold) {
                int num_elements        = std::min(static_cast<int>(forward.error_vector.size()), 10);
                auto min_element_iter   = std::min_element(forward.error_vector.end() - num_elements, forward.error_vector.end());
                subs.forward_error      = *min_element_iter;
            }
        }
        if (!lateral.error_vector.empty()) {
            auto it                     = lateral.error_vector.end();
            if (*it - subs.lateral_error > threshold) {
                int num_elements        = std::min(static_cast<int>(lateral.error_vector.size()), 10);
                auto max_element_iter   = std::max_element(lateral.error_vector.end() - num_elements, lateral.error_vector.end());
                subs.lateral_error      = *max_element_iter;
            }
            else if (*it - subs.lateral_error < -1*threshold) {
                int num_elements        = std::min(static_cast<int>(lateral.error_vector.size()), 10);
                auto min_element_iter   = std::min_element(lateral.error_vector.end() - num_elements, lateral.error_vector.end());
                subs.lateral_error      = *min_element_iter;
            }
        }
        float pid_depth;
        if (!cmd_pwm.arm) {
            prev_time = ros::Time::now().toSec();
            std::cout << "press P for next iteration" << std::endl;
        }
        else {
            if (increasing_h) {
                if (increasing_h_kp && (time_now - prev_time) <= time_interval) {
                    depth.kp    = pid_constants[0] + _h;
                    depth.ki    = pid_constants[1];
                    depth.kd    = pid_constants[2];
                    pid_depth   = depth.pid_control(subs.depth_error, (time_now-init_time), false);
                    cost_h[0]   = cost_h[0] + sqrt(subs.depth_error * subs.depth_error * (dt));
                    std::cout << "Currently calculating cost with kp_h" << std::endl;
                    std::cout   << "current time: " << (time_now - prev_time) << std::endl;
                }
                else if (increasing_h_kp) {
                    increasing_h_kp = false;
                    increasing_h_ki = true;
                    // prev_time = ros::Time::now();
                    //reset
                    cmd_pwm.arm = false;
                    std::cout << "Press p to continue the process _h_kp" << std::endl;
                }
                else if (increasing_h_ki && (time_now - prev_time) <= time_interval) {
                    depth.kp    = pid_constants[0];
                    depth.ki    = pid_constants[1] + _h;
                    depth.kd    = pid_constants[2];
                    pid_depth   = depth.pid_control(subs.depth_error, (time_now-init_time), false);     
                    cost_h[1]   = cost_h[1] + sqrt(subs.depth_error * subs.depth_error * (dt));
                    std::cout << "Currently calculating cost with ki_h" << std::endl;
                    std::cout   << "current time: " << (time_now - prev_time) << std::endl;
                }
                else if (increasing_h_ki) {
                    increasing_h_ki = false;
                    increasing_h_kd = true;
                    // prev_time = ros::Time::now();
                    //reset
                    cmd_pwm.arm = false;
                    std::cout << "Press p to continue the process _h_ki" << std::endl;

                }
                else if (increasing_h_kd && (time_now - prev_time) <= time_interval) {
                    depth.kp    = pid_constants[0];
                    depth.ki    = pid_constants[1];
                    depth.kd    = pid_constants[2] + _h;
                    pid_depth   = depth.pid_control(subs.depth_error, (time_now-init_time), false); 
                    cost_h[2]   = cost_h[2] + sqrt(subs.depth_error * subs.depth_error * (dt));       
                    std::cout   << "Currently calculating cost with kd_h" << std::endl;
                    std::cout   << "current time: " << (time_now - prev_time) << std::endl;
                }
                else if (increasing_h_kd) {
                    increasing_h_kd = false;
                    increasing_h    = false;
                    //reset
                    cmd_pwm.arm = false;
                    std::cout << "Press p to continue the process _h_kd" << std::endl;
                    // prev_time = ros::Time::now();
                    iterations = iterations + 1;
                    pid_constants = descent.execute_adam(pid_constants, cost, cost_h);
                    cost_values.push_back(cost);
                    cost          = 0;
                    cost_h[0]     = 0;
                    cost_h[1]     = 0;
                    cost_h[2]     = 0;
                }
            }
            else {
                if ((time_now - prev_time) >= time_interval) {
                    increasing_h = true;
                    increasing_h_kp = true;
                    // prev_time = ros::Time::now();
                    //reset
                    cmd_pwm.arm = false;
                    std::cout << "Press p to continue the process" << std::endl;
                }
                else {
                    depth.kp    = pid_constants[0];
                    depth.ki    = pid_constants[1];
                    depth.kd    = pid_constants[2];
                    pid_depth   = depth.pid_control(subs.depth_error, (time_now-init_time), false);
                    cost        = cost + sqrt(subs.depth_error * subs.depth_error * (dt));
                    std::cout   << "Cost: " << cost << std::endl;
                    std::cout   << "Currently calculating cost without _h" << std::endl;
                    std::cout   << "current time: " << (time_now - prev_time) << std::endl;
                }
            }
        }
        std::cout << "Iteration Number: " << iterations <<std::endl;
        cmd_pwm.thrust      = pid_depth;
        pwm_publisher.publish(cmd_pwm);
        ros::spinOnce();
        // loop_rate.sleep();
    }
}