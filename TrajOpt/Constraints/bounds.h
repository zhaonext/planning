#pragma once
#include <memory>

#include "Common/common.h"

class Bounds {
public:
    using BoundsConstraintFunction = std::function<void(double* val, const double* x, const double* u)>;
    using BoundsConstraintJacobian = std::function<void(double* jac, const double* x, const double* u)>;

    // u_min - u <= 0
    BoundsConstraintFunction lower_bound_function = [&](double* val, const double* x, const double* u) {
        StateVector state = pointerToStateVector(x);
        InputVector input = pointerToInputVector(u);

        double v_ego = vectorToState(state).v;
        double min_phidot = v_ego * tan(MINIMUM_FRONT_WHEEL_TURNING_ANGLE) / WHEELBASE - 0.5;
        //        printf("min_phidot = %f\n", min_phidot);

        auto lower_bounds = getBoundsLU();
        lower_bounds(1, 0) = min_phidot;

        Eigen::Map<Bounds_su> outputVal(val, NUM_STATE + NUM_INPUT);

        // Lower bounds
        outputVal.head(NUM_STATE) = getBoundsLS() - state;
        outputVal.tail(NUM_INPUT) = lower_bounds - input;

        //        std::cout << "lower bound fun: " << outputVal.transpose() << std::endl;
    };
    BoundsConstraintJacobian lower_bound_jacobian = [&](double* jac, const double* x, const double* u) {
        Eigen::Map<Eigen::MatrixXd> jacobianMatrix(jac, NUM_STATE + NUM_INPUT, NUM_STATE + NUM_INPUT);
        //        jacobianMatrix.setZero();
        jacobianMatrix.diagonal().setConstant(-1.0);

        double l_phidot_dv = tan(MINIMUM_FRONT_WHEEL_TURNING_ANGLE) / WHEELBASE;

        // Lower bound
        jacobianMatrix(NUM_STATE + NUM_INPUT - 1, 5) = l_phidot_dv;

        //        std::cout << "lower bound jac: " << std::endl;
        //        std::cout << jacobianMatrix << std::endl;
    };

    BoundsConstraintFunction upper_bound_function = [&](double* val, const double* x, const double* u) {
        StateVector state = pointerToStateVector(x);
        InputVector input = pointerToInputVector(u);

        double v_ego = vectorToState(state).v;
        double max_phidot = v_ego * tan(MAXIMUM_FRONT_WHEEL_TURNING_ANGLE) / WHEELBASE + 0.5;
        //        printf("max_phidot = %f\n", max_phidot);
        auto upper_bounds = getBoundsUU();
        upper_bounds(1, 0) = max_phidot;

        Eigen::Map<Bounds_su> outputVal(val, NUM_STATE + NUM_INPUT);

        // upper bounds
        outputVal.head(NUM_STATE) = state - getBoundsUS();
        outputVal.tail(NUM_INPUT) = input - upper_bounds;

        //        std::cout << "upper bound fun: " << outputVal.transpose() << std::endl;
    };
    BoundsConstraintJacobian upper_bound_jacobian = [&](double* jac, const double* x, const double* u) {
        Eigen::Map<Eigen::MatrixXd> jacobianMatrix(jac, NUM_STATE + NUM_INPUT, NUM_STATE + NUM_INPUT);
        //        jacobianMatrix.setZero();
        jacobianMatrix.diagonal().setConstant(1.0);

        double u_phidot_dv = tan(MAXIMUM_FRONT_WHEEL_TURNING_ANGLE) / WHEELBASE;

        // upper bound
        jacobianMatrix(NUM_STATE + NUM_INPUT - 1, 5) = -u_phidot_dv;

        //        std::cout << "upper bound jac: " << std::endl;
        //        std::cout << jacobianMatrix << std::endl;
    };

public:
    Bounds() {
        l_bounds_s_(0) = -INF;                    // s
        l_bounds_s_(1) = MINIMUM_LATERAL_OFFSET;  // n
        l_bounds_s_(2) = -INF;                    // alpha
        l_bounds_s_(3) = -INF;                    // x
        l_bounds_s_(4) = -INF;                    // y
        l_bounds_s_(5) = MINIMUM_SPEED;           // v
        l_bounds_s_(6) = -INF;                    // phi
        l_bounds_u_(0) = MINIMUM_ACCELERATION;    // acc
        l_bounds_u_(1) = -INF;                    // phidot

        u_bounds_s_(0) = INF;
        u_bounds_s_(1) = MAXIMUM_LATERAL_OFFSET;
        u_bounds_s_(2) = INF;
        u_bounds_s_(3) = INF;
        u_bounds_s_(4) = INF;
        u_bounds_s_(5) = MAXIMUM_SPEED;
        u_bounds_s_(6) = INF;
        u_bounds_u_(0) = MAXIMUM_ACCELERATION;
        u_bounds_u_(1) = INF;
    }

    Bounds_s getBoundsLS() { return l_bounds_s_; }
    Bounds_u getBoundsLU() { return l_bounds_u_; }

    Bounds_s getBoundsUS() { return u_bounds_s_; }
    Bounds_u getBoundsUU() { return u_bounds_u_; }

private:
    Bounds_s l_bounds_s_;
    Bounds_u l_bounds_u_;

    Bounds_s u_bounds_s_;
    Bounds_u u_bounds_u_;
};
