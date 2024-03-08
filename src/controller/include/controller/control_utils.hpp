#include <ros/ros.h> 
#include <geometry_msgs/Vector3.h> 
#include <cmath> 

/**
 * @brief Control class for implementing PID control.
 */
class Control {
public:
    float pwm, kp, kd, ki, current_velocity, prev_velocity = 0, pwm_prev = 1500; // Control parameters and variables

    /**
     * @brief Function to compute PID control.
     * 
     * @param error Error value for PID control
     * @param dtime Time difference for PID control
     * @return float PWM value computed using PID control
     */
    float pid_control(float error, float dtime) {
        // Output yaw error to console
        std::cout << "yaw error: " << error << std::endl;

        // Store time and error values
        time.push_back(dtime);
        error_vector.push_back(error);

        // Compute proportional, integral, and derivative components of PID control
        float pid_p = (kp * error);
        float pid_i = (integrate(error_vector, time));
        float pid_d = 0;
        try {
            pid_d = (kd * (error_vector[error_vector.size() - 1] - error_vector[error_vector.size() - 2]));
        } catch (...) {
            pid_d = 0;
        }

        // Limit integral term to prevent windup
        if (pid_i > std::max(180 - pid_p - pid_d, float(0.0))) {
            pid_i = ki * (std::max(180 - pid_p - pid_d, float(0.0)));
        } else if (pid_i < std::max(-180 - pid_p - pid_d, float(0.0))) {
            pid_i = ki * (std::max(-180 - pid_p - pid_d, float(0.0)));
        } else {
            pid_i = ki * pid_i;
        }

        // Compute PWM value
        pwm = pid_p + pid_d + pid_i;

        // Limit PWM value to range [-180, 180]
        if (pwm > 180) {
            pwm = 180;
        } else if (pwm < -180) {
            pwm = -180;
        }

        // Convert PWM value to pulse width range [1100, 1900]
        pwm = 1900 - (pwm + 180) * (800) / 360;

        // Adjust PWM value to prevent sudden changes
        if (sqrt((pwm_prev - pwm) * (pwm_prev - pwm)) > 200) {
            if (pwm_prev > pwm) {
                pwm = pwm_prev - 10;
                pwm_prev = pwm;
                return pwm;
            } else {
                pwm = pwm_prev + 10;
                pwm_prev = pwm;
                return pwm;
            }
        }
        pwm_prev = pwm;
        return pwm;
    }

    /**
     * @brief Function to return a hold value.
     * 
     * @return float Hold value (1500)
     */
    float hold() {
        return 1500.00;
    }

private:
    std::vector<double> time; // Vector to store time values
    std::vector<double> error_vector; // Vector to store error values

    /**
     * @brief Function to perform trapezoidal integration.
     * 
     * @param error Vector of error values
     * @param time Vector of time values
     * @return float Integrated value
     */
    float integrate(std::vector<double> error, std::vector<double> time) {
        // Trapezoidal integration
        return 0;
    }
};

