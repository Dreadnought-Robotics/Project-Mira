#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <vector>
#include <algorithm>

class Control
{
public:
    float pwm, kp, kd, ki, current_velocity;
    float prev_velocity = 0, pwm_prev = 1500, prev_d = 0;
    std::vector<float> error_vector;

    /**
     * @brief PID control function to compute the PWM signal based on the error.
     *
     * @param error The current error value.
     * @param dtime The time difference since the last call.
     * @param switch_polarity Boolean to switch the polarity of the PWM signal.
     * @return Computed PWM signal.
     */
    float pid_control(float error, float dtime, bool switch_polarity)
    {
        time.push_back(dtime);
        error_vector.push_back(error);

        float pid_p = kp * error;
        float pid_i = integrate(error_vector, time);
        float pid_d = 0;

        try
        {
            if (error_vector.size() > 1 && error_vector[error_vector.size() - 1] == error_vector[error_vector.size() - 2])
            {
                pid_d = prev_d;
            }
            else
            {
                pid_d = kd * (error_vector[error_vector.size() - 1] - error_vector[error_vector.size() - 2]);
            }
        }
        catch (...)
        {
            pid_d = 0;
        }

        pid_i = ki * pid_i;
        pid_i = std::clamp(pid_i, std::max(-180 - pid_p - pid_d, float(0.0)), std::max(180 - pid_p - pid_d, float(0.0)));

        pwm = pid_p + pid_d + pid_i;
        pwm = std::clamp(pwm, -180.0f, 180.0f);

        if (switch_polarity)
        {
            pwm = pwm < 0 ? 1100 + (pwm + 180) * 384 / 180 : 1900 - (-pwm + 180) * 384 / 180;
        }
        else
        {
            pwm = pwm < 0 ? 1900 - (pwm + 180) * 384 / 180 : 1100 + (-pwm + 180) * 384 / 180;
        }

        prev_d = pid_d;
        pwm_prev = pwm;

        return pwm;
    }

    /**
     * @brief Returns a neutral PWM signal value.
     *
     * @return Neutral PWM signal.
     */
    float hold()
    {
        return 1500.00;
    }

    /**
     * @brief Clears the error and time vectors.
     */
    void emptyError()
    {
        error_vector.clear();
        time.clear();
    }

private:
    std::vector<float> time;

    /**
     * @brief Integrates the error over time to compute the integral part of the PID controller.
     *
     * @param error Vector of error values.
     * @param time Vector of time intervals.
     * @return Integral of the error over time.
     */
    float integrate(const std::vector<float> &error, const std::vector<float> &time)
    {
        float area = 0;

        try
        {
            for (size_t i = 1; i < time.size(); ++i)
            {
                area += (time[i] - time[i - 1]) * (error[i] + error[i - 1]) / 2;
            }
        }
        catch (...)
        {
            // Handle integration errors if necessary
        }

        return area;
    }
};
