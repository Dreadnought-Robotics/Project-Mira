#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <vector>

/**
 * @brief Control class for managing PID control of an AUV (Autonomous Underwater Vehicle).
 */
class Control {
public:
    float pwm; ///< PWM output
    float kp, kd, ki; ///< PID coefficients
    float current_velocity, prev_velocity = 0;
    float pwm_prev = 1500;
    float prev_d = 0;
    std::vector<float> error_vector; ///< Error history

    /**
     * @brief Performs PID control and calculates the PWM output.
     * 
     * @param error Current error value.
     * @param dtime Delta time since last control step.
     * @param switch_polarity Flag to switch polarity of the control signal.
     * @return float Calculated PWM value.
     */
    float pid_control(float error, float dtime, bool switch_polarity) {
        time.push_back(dtime);
        error_vector.push_back(error);

        // Proportional term
        float pid_p = kp * error;

        // Integral term
        float pid_i = integrate(error_vector, time) * ki;
        
        // Derivative term
        float pid_d = 0;
        try {
            if (error_vector.size() > 1 && error_vector.back() == error_vector[error_vector.size() - 2]) {
                pid_d = prev_d;
            } else {
                pid_d = kd * (error_vector.back() - error_vector[error_vector.size() - 2]);
            }
        } catch (...) {
            pid_d = 0;
        }

        // Clamping integral term to prevent windup
        pid_i = std::clamp(pid_i, std::max(-180 - pid_p - pid_d, 0.0f), std::max(180 - pid_p - pid_d, 0.0f));

        pwm = pid_p + pid_d + pid_i;

        // Clamping PWM output
        pwm = std::clamp(pwm, -180.0f, 180.0f);

        // Convert PWM to appropriate range based on polarity
        if (switch_polarity) {
            pwm = pwm < 0 ? 1100 + (pwm + 180) * 384 / 180 : 1900 - (-pwm + 180) * 384 / 180;
        } else {
            pwm = pwm < 0 ? 1900 - (pwm + 180) * 384 / 180 : 1100 + (-pwm + 180) * 384 / 180;
        }

        prev_d = pid_d;
        pwm_prev = pwm;

        return pwm;
    }

    /**
     * @brief Returns the hold PWM value.
     * 
     * @return float Hold PWM value.
     */
    float hold() {
        return 1500.0;
    }

    /**
     * @brief Clears the error and time history.
     */
    void emptyError() {
        error_vector.clear();
        time.clear();
    }

private:
    std::vector<float> time; ///< Time history

    /**
     * @brief Integrates the error over time.
     * 
     * @param error Error values.
     * @param time Time values.
     * @return float Integrated error value.
     */
    float integrate(const std::vector<float>& error, const std::vector<float>& time) {
        float area = 0;
        try {
            for (size_t i = 1; i < time.size(); ++i) {
                area += (time[i] - time[i - 1]) * (error[i] + error[i - 1]) / 2;
            }
        } catch (...) {
            // Handle exceptions silently
        }
        return area;
    }
};
