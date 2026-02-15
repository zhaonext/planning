#pragma once

#include "Common/common.h"
#include "Map/curve.h"
#include "PathSegment/path.h"

class Model {
public:
    explicit Model(const std::shared_ptr<Path> &curve) : ref_path_(curve){};

    StateVector dynamics(const State &X, const Input &U);
    Eigen::MatrixXd getModelJacobian(const State &X, const Input &U);

private:
    std::shared_ptr<Path> ref_path_;
};
