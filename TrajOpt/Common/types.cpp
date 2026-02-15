#include "types.h"

StateVector stateToVector(const State &s) {
    StateVector X;
    X(0) = s.s;
    X(1) = s.n;
    X(2) = s.alpha;
    X(3) = s.x;
    X(4) = s.y;
    X(5) = s.v;
    X(6) = s.phi;

    return X;
}

InputVector inputToVector(const Input &U) {
    InputVector u = {U.acceleration, U.phidot};
    return u;
}

State vectorToState(const StateVector &X) {
    State x{};
    x.setZero();
    x.s = X(0);
    x.n = X(1);
    x.alpha = X(2);
    x.x = X(3);
    x.y = X(4);
    x.v = X(5);
    x.phi = X(6);

    return x;
}

Input vectorToInput(const InputVector &uk) {
    Input u{};
    u.setZero();
    u.acceleration = uk(0);
    u.phidot = uk(1);

    return u;
}

State pointerToState(const double *X) {
    State x{};
    x.s = X[0];
    x.n = X[1];
    x.alpha = X[2];
    x.x = X[3];
    x.y = X[4];
    x.v = X[5];
    x.phi = X[6];

    return x;
}
Input pointerToInput(const double *U) {
    Input u{};
    u.acceleration = U[0];
    u.phidot = U[1];

    return u;
}

StateVector pointerToStateVector(const double *X) {
    State state = pointerToState(X);
    return stateToVector(state);
}

InputVector pointerToInputVector(const double *U) {
    Input input = pointerToInput(U);
    return inputToVector(input);
};

EgoVehicleInfo StateToEgoVehicleInfo(const State &state) {
    EgoVehicleInfo ego_info;
    ego_info.x = state.x;
    ego_info.y = state.y;
    ego_info.v = state.v;
    ego_info.phi = state.phi;

    return ego_info;
}

Vec2d StateToVec2d(const State &state) {
    Vec2d vec;
    vec.set_x(state.x);
    vec.set_y(state.y);

    return vec;
}
