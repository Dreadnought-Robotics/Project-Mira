#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class Control {
    public:
        float pwm, kp ,kd, ki, current_velocity, prev_velocity=0, pwm_prev=1500;
        float pid_control(float error, float dtime) {
            std::cout << "yaw error: " << error << std::endl;
            time.push_back(dtime);
            error_vector.push_back(error);
            float pid_p = (kp*error);
            float pid_i = (integrate(error_vector, time));
            float pid_d = 0;
            // float pid_d = (kd*current_velocity);
            try {
                pid_d       = (kd*(error_vector[error_vector.size()-1] - error_vector[error_vector.size()-2]));
            }
            catch(...) {
                pid_d       = 0;
            }
            if (pid_i > std::max(180-pid_p-pid_d, float(0.0))) {
                pid_i       = ki*(std::max(180-pid_p-pid_d, float(0.0)));
            }
            else if (pid_i < std::max(-180-pid_p-pid_d, float(0.0))) {
                pid_i       = ki*(std::max(-180-pid_p-pid_d, float(0.0)));
            }
            else {
                pid_i       = ki*pid_i;
            }
            // prev_velocity   = current_velocity;
            pwm             = pid_p + pid_d + pid_i;         

            if (pwm>180) {
                pwm         = 180;
            }       
            else if(pwm<-180) {
                pwm         = -180;
            }
            pwm             = 1900 - (pwm+180)*(800)/360;
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
            pwm_prev = pwm;
            return pwm;
        }
        float hold() {
            return 1500.00;
        }
    private:
    std::vector<double> time;
    std::vector<double> error_vector;
        float integrate(std::vector<double> error, std::vector<double> time) {
            // trapezoidal integration
            return 0;
        }
};