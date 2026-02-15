#pragma once
#include <memory>
#include <utility>

#include "Common/common.h"
#include "PathSegment/path.h"
#include "model.h"

class Integrator {
public:
    using DynamicsFunction = std::function<void(double *odes, const double *x, const double *u, float h)>;
    using DynamicsJacobian = std::function<void(double *jac, const double *x, const double *u, float h)>;

    DynamicsFunction dynamics_function = [&](double *xn, const double *x, const double *u, float h) -> void {
        State state = pointerToState(x);
        Input input = pointerToInput(u);

        State next_state = RK4(state, input, h);
        //        State next_state = ForwardEuler(state, input, h);
        //        State next_state = MidpointEuler(state, input, h);
        //        State next_state = BackwardEuler(state, input, h);

        //        std::cout << "error_state: " << (stateToVector(next_state1) - stateToVector(next_state)).transpose() << std::endl;

        StateVector x_next_vec = stateToVector(next_state);
        std::copy(x_next_vec.data(), x_next_vec.data() + NUM_STATE, xn);
    };
    DynamicsJacobian dynamics_jacobian = [&](double *jac, const double *x, const double *u, float h) -> void {
        State state = pointerToState(x);
        Input input = pointerToInput(u);

        // 静态变量定义
        static Eigen::MatrixXd A(NUM_STATE, NUM_STATE);
        static Eigen::MatrixXd B(NUM_STATE, NUM_INPUT);
        static Eigen::MatrixXd Am(NUM_STATE, NUM_STATE);
        static Eigen::MatrixXd Bm(NUM_STATE, NUM_INPUT);
        static StateVector xm(NUM_STATE);
        static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(NUM_STATE, NUM_STATE);

        Eigen::Map<Eigen::MatrixXd> J(jac, NUM_STATE, NUM_STATE + NUM_INPUT);

        // 评估中点
        StateVector x_vec = stateToVector(state);
        StateVector f = model_.dynamics(state, input);
        xm = x_vec + h / 2 * f;

        // 当前状态评估雅可比矩阵
        Eigen::MatrixXd continuous_jacobian = model_.getModelJacobian(state, input);
        A = continuous_jacobian.leftCols(NUM_STATE);
        B = continuous_jacobian.rightCols(NUM_INPUT);

        // 在中点评估雅可比矩阵
        continuous_jacobian = model_.getModelJacobian(vectorToState(xm), input);
        Am = continuous_jacobian.leftCols(NUM_STATE);
        Bm = continuous_jacobian.rightCols(NUM_INPUT);

        // 中点离散化
        J.leftCols(NUM_STATE) = In + h * Am * (In + h / 2 * A);
        J.rightCols(NUM_INPUT) = h * (Am * h / 2 * B + Bm);
    };

public:
    explicit Integrator(const std::shared_ptr<Path> &curve) : model_(curve) {}
    explicit Integrator(Model model) : model_(std::move(model)) {}

    State RK4(const State &x, const Input &u, double ts);
    State ForwardEuler(const State &x, const Input &u, double ts);
    State BackwardEuler(const State &x, const Input &u, double ts);
    State MidpointEuler(const State &x, const Input &u, double ts);

private:
    Model model_;
};
