#include "integrator.h"

State Integrator::RK4(const State &x, const Input &u, const double ts) {
    // 4th order Runge Kutta (RK4) implementation
    const StateVector x_vec = stateToVector(x);
    const InputVector u_vec = inputToVector(u);

    // evaluating the 4 points
    const StateVector k1 = model_.dynamics(vectorToState(x_vec), u);
    const StateVector k2 = model_.dynamics(vectorToState(x_vec + ts / 2. * k1), u);
    const StateVector k3 = model_.dynamics(vectorToState(x_vec + ts / 2. * k2), u);
    const StateVector k4 = model_.dynamics(vectorToState(x_vec + ts * k3), u);

    const StateVector x_next = x_vec + ts * (k1 / 6. + k2 / 3. + k3 / 3. + k4 / 6.);
    return vectorToState(x_next);
}

State Integrator::ForwardEuler(const State &x, const Input &u, double ts) {
    const StateVector x_vec = stateToVector(x);
    const StateVector f = model_.dynamics(x, u);

    const StateVector x_next = x_vec + ts * f;
    return vectorToState(x_next);
}
State Integrator::MidpointEuler(const State &x, const Input &u, double ts) {
    // 将当前状态转换为向量
    const StateVector x_vec = stateToVector(x);

    // 计算中点处的状态
    StateVector x_mid = x_vec + (ts / 2.0) * model_.dynamics(x, u);

    // 使用中点处的斜率来更新状态
    const StateVector x_next = x_vec + ts * model_.dynamics(vectorToState(x_mid), u);

    return vectorToState(x_next);
}
State Integrator::BackwardEuler(const State &x, const Input &u, double ts) {
    const int MAX_ITERATIONS = 100;  // 迭代的最大次数
    const double tolerance = 1e-6;   // 收敛的阈值

    StateVector x_next_vec = stateToVector(x);
    StateVector x_next_vec_old;
    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        x_next_vec_old = x_next_vec;
        StateVector f_next = model_.dynamics(vectorToState(x_next_vec), u);
        x_next_vec = stateToVector(x) + ts * f_next;

        if ((x_next_vec - x_next_vec_old).norm() < tolerance) {
            break;
        }
    }

    return vectorToState(x_next_vec);
}
