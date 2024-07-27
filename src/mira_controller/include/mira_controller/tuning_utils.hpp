#include <iostream>
#include <vector>
#include <cmath>

class GradientDescent
{
private:
    double a_min;
    double a_max;
    std::vector<double> points;
    std::vector<double> result;
    std::vector<std::vector<double>> G{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double beta1;
    double beta2;
    double epsilon;
    std::vector<double> m;
    std::vector<double> v;
    int t;

public:
    std::vector<double> learning_rate;

    /**
     * @brief Constructor for GradientDescent.
     *
     * @param learning_rate Vector of learning rates for each parameter.
     * @param a_min Minimum value for parameters.
     * @param a_max Maximum value for parameters.
     * @param beta1 Decay rate for the first moment estimate (Adam optimizer).
     * @param beta2 Decay rate for the second moment estimate (Adam optimizer).
     * @param epsilon Small value to prevent division by zero (Adam optimizer).
     */
    GradientDescent(std::vector<double> learning_rate, double a_min = 0, double a_max = 0, double beta1 = 0.9, double beta2 = 0.999, double epsilon = 1e-8)
        : learning_rate(learning_rate), a_min(a_min), a_max(a_max), beta1(beta1), beta2(beta2), epsilon(epsilon), t(0) {}

    /**
     * @brief Computes the gradient of the cost function.
     *
     * @param control_variable Current control variables.
     * @param current_cost Current cost.
     * @param cost_at_increased_h Cost at increased control variables by a small h.
     * @return Gradient vector.
     */
    std::vector<double> grad(std::vector<double> control_variable, double current_cost, std::vector<double> cost_at_increased_h)
    {
        double h = 0.0000001;
        std::vector<double> grad;
        grad.reserve(control_variable.size());
        for (int i = 0; i < control_variable.size(); ++i)
        {
            grad.push_back((cost_at_increased_h[i] - current_cost) / h);
        }
        return grad;
    }

    /**
     * @brief Updates the control variables using gradient descent.
     *
     * @param current_cost Current cost.
     * @param grad Gradient vector.
     */
    void update_a(std::vector<double> &current_cost, std::vector<double> grad)
    {
        for (int i = 0; i < grad.size(); ++i)
        {
            current_cost[i] -= learning_rate[i] * grad[i];
            if ((a_min != 0.0) || (a_max != 0.0))
            {
                current_cost[i] = std::min(std::max(current_cost[i], a_min), a_max);
            }
        }
    }

    /**
     * @brief Updates the control variables using the Adam optimization algorithm.
     *
     * @param grad Gradient vector.
     * @param a Control variables.
     */
    void update_adam(std::vector<double> grad, std::vector<double> &a)
    {
        ++t;
        if (m.empty())
        {
            m.resize(grad.size(), 0.0);
            v.resize(grad.size(), 0.0);
        }
        for (int i = 0; i < grad.size(); ++i)
        {
            m[i] = beta1 * m[i] + (1 - beta1) * grad[i];
            v[i] = beta2 * v[i] + (1 - beta2) * (grad[i] * grad[i]);
            double m_hat = m[i] / (1 - std::pow(beta1, t));
            double v_hat = v[i] / (1 - std::pow(beta2, t));
            learning_rate[i] = learning_rate[i] * (1 / (sqrt(v_hat) + epsilon));
            a[i] -= learning_rate[i] * m_hat;
        }
    }

    /**
     * @brief Updates the matrix G used in the AdaGrad optimization algorithm.
     *
     * @param grad Gradient vector.
     */
    void update_G(std::vector<double> grad)
    {
        for (int i = 0; i < grad.size(); ++i)
        {
            for (int j = 0; j < grad.size(); ++j)
            {
                G[i][j] += grad[i] * grad[j];
            }
        }
    }

    /**
     * @brief Executes one iteration of the AdaGrad optimization algorithm.
     *
     * @param a Control variables.
     * @param cost_function Current cost.
     * @param cost_function_a_h Cost at increased control variables by a small h.
     * @return Updated control variables.
     */
    std::vector<double> execute_adagrad(std::vector<double> a, double cost_function, std::vector<double> cost_function_a_h)
    {
        points = a;
        result.push_back(cost_function);
        auto grad = this->grad(a, cost_function, cost_function_a_h);
        update_G(grad);
        for (int i = 0; i < grad.size(); ++i)
        {
            learning_rate[i] *= 1 / std::sqrt(G[i][i]);
        }
        update_a(a, grad);
        return a;
    }

    /**
     * @brief Executes one iteration of the Adam optimization algorithm.
     *
     * @param a Control variables.
     * @param cost_function Current cost.
     * @param cost_function_a_h Cost at increased control variables by a small h.
     * @return Updated control variables.
     */
    std::vector<double> execute_adam(std::vector<double> a, double cost_function, std::vector<double> cost_function_a_h)
    {
        points = a;
        result.push_back(cost_function);
        auto grad = this->grad(a, cost_function, cost_function_a_h);
        update_adam(grad, a);
        return a;
    }
};
