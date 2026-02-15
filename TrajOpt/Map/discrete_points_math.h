#pragma once

#include <utility>
#include <vector>

#include "Common/common.h"
#include "boost/math/tools/minima.hpp"
class DiscretePoints {
public:
    explicit DiscretePoints(const std::vector<std::pair<double, double>>& xy_points) : xy_points_(xy_points) {
        ComputePathProfile();
    };

    bool ComputePathProfile();
    double GetArcLengthToClosestPoint(double x, double y);
    double FindClosestPointIndex(double x, double y);
    std::pair<double, double> GetClosestPoint(double x, double y);
    double GetHeadingAtClosestPoint(double x, double y);

    const std::vector<double>& getHeadings() const { return headings_; }
    const std::vector<double>& getAccumulatedS() const { return accumulated_s_; }
    const std::vector<double>& getKappas() const { return kappas_; }
    const std::vector<double>& getDkappas() const { return dkappas_; }

private:
    std::vector<std::pair<double, double>> xy_points_;
    std::vector<double> headings_;
    std::vector<double> accumulated_s_;
    std::vector<double> kappas_;
    std::vector<double> dkappas_;
};
