#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class Control {
    public:
        float pwm, kp ,kd, ki, current_velocity, prev_velocity=0, pwm_prev=1500, prev_d = 0;
        std::vector<float> error_vector;
        float pid_control(float error, float dtime, bool switch_polarity) {
            time.push_back(dtime);
            error_vector.push_back(error);
            float pid_p = (kp*error);
            float pid_i = (integrate(error_vector, time));
            float pid_d = 0;
            // float pid_d = (kd*current_velocity);
            try {
                if (error_vector[error_vector.size()-1] == error_vector[error_vector.size()-2]) {
                    pid_d = prev_d;
                }
                else {
                    pid_d       = (kd*(error_vector[error_vector.size()-1] - error_vector[error_vector.size()-2]));
                }
            }
            catch(...) {
                pid_d       = 0;
            }
            pid_i = ki*pid_i;
            if (pid_i > std::max(180-pid_p-pid_d, float(0.0))) {
                pid_i       = (std::max(180-pid_p-pid_d, float(0.0)));
            }
            else if (pid_i < std::max(-180-pid_p-pid_d, float(0.0))) {
                pid_i       = (std::max(-180-pid_p-pid_d, float(0.0)));
            }
            else {
                pid_i       = pid_i;
            }
            // prev_velocity   = current_velocity;
            pwm             = pid_p + pid_d + pid_i;         
            // std::cout << "Error: " << error << ", p: " << pid_p << ", i: " << pid_i << ", d: " << pid_d << std::endl;
            if (pwm>180) {
                pwm         = 180;
            }       
            else if(pwm<-180) {
                pwm         = -180;
            }
            if (switch_polarity) {
                if (pwm<0) {
                pwm             = 1100 + (pwm+180)*(384)/180;
                }
                else {
                pwm             = 1900 - (-1*pwm+180)*(384)/180;
                }
                // pwm             = 1100 + (pwm+180)*(800)/360;
            }  
            else {
                if (pwm<0) {
                pwm             = 1900 - (pwm+180)*(384)/180;
                }
                else {
                pwm             = 1100 + (-1*pwm+180)*(384)/180;
                }
                // pwm             = 1900 - (pwm+180)*(800)/360;
            }
            // pwm             = 1100 + (pwm+180)*(800)/360;
            if (sqrt((pwm_prev - pwm)*(pwm_prev - pwm)) > 200) {
                if (pwm_prev > pwm) {
                    pwm = pwm_prev-10;
                    pwm_prev = pwm;
                    return pwm;
                }
                else {
                    pwm = pwm_prev+10;
                    pwm_prev = pwm;
                    return pwm;
                }
            }
            prev_d = pid_d;
            pwm_prev = pwm;
            // std::cout << "Error stack: " << error_vector.size() << std::endl;
            return pwm;
        }
        float hold() {
            return 1500.00;
        }
        void emptyError() {
            // for (int i=0; i<error_vector.size(); i++) {
            while (error_vector.size()!=0){
                error_vector.pop_back();
                time.pop_back();
            }
            std::cout << "Error stack: " << error_vector.size() << std::endl;
        }
    private:
    std::vector<float> time;
        float integrate(std::vector<float> error, std::vector<float> time) {
            float area = 0;
            // try {
            //     for (int i = time.size()-2000000; i < time.size(); i++) {
            //         area += (time[i] - time[i-1]) * (error[i] + error[i-1]) / 2;
            //     }                    
            // }
            // catch(...) {
                try {
                    for (int i = 1; i < time.size(); i++) {
                        area += (time[i] - time[i-1]) * (error[i] + error[i-1]) / 2;
                    }
                }
                catch(...){
                }
            // }
            
            // std::cout << area << std::endl;
            
            return area;
        }

};