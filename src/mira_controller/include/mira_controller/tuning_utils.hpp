#include <iostream>
#include <vector>
#include <cmath>

class GradientDescent {
private:
    double learning_rate;
    double a_min;
    double a_max;
    std::vector<double> points;
    std::vector<double> result;
    std::vector<std::vector<double>> G;
    double beta1;
    double beta2;
    double epsilon;
    std::vector<double> m;
    std::vector<double> v;
    int t;

public:
    GradientDescent(double learning_rate, double a_min = 0.0, double a_max = 0.0, double beta1 = 0.9, double beta2 = 0.999, double epsilon = 1e-8)
        : learning_rate(learning_rate), a_min(a_min), a_max(a_max), beta1(beta1), beta2(beta2), epsilon(epsilon), t(0) {}
    
    std::vector<double> grad(std::vector<double> control_variable, double current_cost, std::vector<double> cost_at_increased_h) {
        double h                = 0.0000001;
        std::vector<double>     grad;
        grad.reserve(control_variable.size());
        for (int i = 0; i < control_variable.size(); ++i) {
            grad.push_back((cost_at_increased_h[i] - current_cost) / h);
        }
        return grad;
    }

    void update_a(double learning_rate, std::vector<double>& current_cost, std::vector<double> grad) {
        for (int i = 0; i < grad.size(); ++i) {
            current_cost[i] -= learning_rate * grad[i];
            if ((a_min != 0.0) || (a_max != 0.0)) {
                current_cost[i] = std::min(std::max(current_cost[i], a_min), a_max);
            }
        }
    }

    void update_adam(std::vector<double> grad, std::vector<double>& a) {
        ++t;
        if (m.empty()) {
            m.resize(grad.size(), 0.0);
            v.resize(grad.size(), 0.0);
        }
        for (int i = 0; i < grad.size(); ++i) {
            m[i] = m[i] * beta1 + (1 - beta1) * grad[i];
            v[i] = v[i] * beta2 + (1 - beta2) * (grad[i] * grad[i]);
            double m_hat = m[i] / (1 - std::pow(beta1, t));
            double v_hat = v[i] / (1 - std::pow(beta2, t));
            a[i] -= learning_rate * m_hat / (std::sqrt(v_hat) + epsilon);
        }
    }

    std::vector<double> execute(std::vector<double> a, double cost_function, std::vector<double> cost_function_a_h) {
        this->G.resize(a.size(), std::vector<double>(a.size(), 0.0));
        points = a;
        result.push_back(cost_function);
        auto grad = this->grad(a, cost_function, cost_function_a_h);
        update_a(learning_rate, a, grad);
        return a;
    }

    void update_G(std::vector<double> grad) {
        for (int i = 0; i < grad.size(); ++i) {
            for (int j = 0; j < grad.size(); ++j) {
                G[i][j] += grad[i] * grad[j];
            }
        }
    }

    std::vector<double> execute_adagrad(std::vector<double> a, double cost_function, std::vector<double> cost_function_a_h) {
        points = a;
        result.push_back(cost_function);
        auto grad = this->grad(a, cost_function, cost_function_a_h);
        update_G(grad);
        for (int i = 0; i < grad.size(); ++i) {
            learning_rate *= 1 / std::sqrt(G[i][i]);
        }
        update_a(learning_rate, a, grad);
        return a;
    }

    std::vector<double> execute_adam(std::vector<double> a, double cost_function, std::vector<double> cost_function_a_h) {
        points = a;
        result.push_back(cost_function);
        auto grad = this->grad(a, cost_function, cost_function_a_h);
        update_adam(grad, a);
        return a;
    }

};