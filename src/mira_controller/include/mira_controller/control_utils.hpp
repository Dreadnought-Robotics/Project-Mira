#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <fftw3.h>

class Control {
    public:
        float pwm, kp ,kd, ki, current_velocity, prev_velocity=0, pwm_prev=1500;
        std::vector<double> error_vector;
        float pid_control(float error, float dtime, bool switch_polarity) {
            time.push_back(dtime);
            error_vector.push_back(error);
            error_vector = denoise(error_vector);
            error = error_vector.back();
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
            if (switch_polarity) {
                // if (pwm<0) {
                // pwm             = 1100 + (pwm+180)*(370)/180;
                // }
                // else {
                // pwm             = 1900 - (-1*pwm+180)*(370)/180;
                // }
                pwm             = 1100 + (pwm+180)*(800)/360;
            }  
            else {
                // if (pwm<0) {
                // pwm             = 1900 - (pwm+180)*(370)/180;
                // }
                // else {
                // pwm             = 1100 + (-1*pwm+180)*(370)/180;
                // }
                pwm             = 1900 - (pwm+180)*(800)/360;
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
            pwm_prev = pwm;
            return pwm;
        }
        float hold() {
            return 1500.00;
        }
        void emptyError() {
            for (int i=0; i<error_vector.size(); i++) {
                error_vector.pop_back();
                time.pop_back();
            }
        }
    private:
    std::vector<double> time;
        float integrate(std::vector<double> error, std::vector<double> time) {
            float area = 0;
            try {
                for (int i = 1; i < time.size(); i++) {
                    area += (time[i] - time[i-1]) * (error[i] + error[i-1]) / 2;
                }
            }
            catch(...) {
            }
            
            return area;
        }
    std::vector<double> denoise(const std::vector<double>& data) {
        std::vector<double> result;
        
        if (data.empty())
            return result;

        int N = data.size();

        // Allocate memory for FFT input and output
        fftw_complex *in, *out;
        in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
        out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);

        // Create FFTW plan
        fftw_plan plan_forward = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
        fftw_plan plan_backward = fftw_plan_dft_1d(N, out, in, FFTW_BACKWARD, FFTW_ESTIMATE);

        // Copy data to FFT input
        for (int i = 0; i < N; ++i) {
            in[i][0] = data[i];
            in[i][1] = 0.0;
        }

        // Perform forward FFT
        fftw_execute(plan_forward);

        // Denoise by setting higher frequency components to zero
        const int threshold_index = 10;
        for (int i = threshold_index; i < N; ++i) {
            out[i][0] = 0.0;
            out[i][1] = 0.0;
        }

        // Perform backward FFT
        fftw_execute(plan_backward);

        // Copy result to output vector
        result.resize(N);
        for (int i = 0; i < N; ++i) {
            result[i] = in[i][0] / N;
        }

        // Clean up
        fftw_destroy_plan(plan_forward);
        fftw_destroy_plan(plan_backward);
        fftw_free(in);
        fftw_free(out);

    return result;
}
};