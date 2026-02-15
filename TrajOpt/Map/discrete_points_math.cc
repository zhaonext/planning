#include "discrete_points_math.h"

#include <cmath>

#include "Common/common.h"

bool DiscretePoints::ComputePathProfile() {
    headings_.clear();
    kappas_.clear();
    dkappas_.clear();

    if (xy_points_.size() < 2) {
        return false;
    }
    std::vector<double> dxs;
    std::vector<double> dys;
    std::vector<double> y_over_s_first_derivatives;
    std::vector<double> x_over_s_first_derivatives;
    std::vector<double> y_over_s_second_derivatives;
    std::vector<double> x_over_s_second_derivatives;

    // Get finite difference approximated dx and dy for heading and kappa
    // calculation
    std::size_t points_size = xy_points_.size();
    for (std::size_t i = 0; i < points_size; ++i) {
        double x_delta = 0.0;
        double y_delta = 0.0;
        if (i == 0) {
            x_delta = (xy_points_[i + 1].first - xy_points_[i].first);
            y_delta = (xy_points_[i + 1].second - xy_points_[i].second);
        } else if (i == points_size - 1) {
            x_delta = (xy_points_[i].first - xy_points_[i - 1].first);
            y_delta = (xy_points_[i].second - xy_points_[i - 1].second);
        } else {
            x_delta = 0.5 * (xy_points_[i + 1].first - xy_points_[i - 1].first);
            y_delta = 0.5 * (xy_points_[i + 1].second - xy_points_[i - 1].second);
        }
        dxs.push_back(x_delta);
        dys.push_back(y_delta);
    }

    // Heading calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        headings_.push_back(std::atan2(dys[i], dxs[i]));
    }

    // Get linear interpolated s for dkappa calculation
    double distance = 0.0;
    accumulated_s_.push_back(distance);
    double fx = xy_points_[0].first;
    double fy = xy_points_[0].second;
    double nx = 0.0;
    double ny = 0.0;
    for (std::size_t i = 1; i < points_size; ++i) {
        nx = xy_points_[i].first;
        ny = xy_points_[i].second;
        double end_segment_s =
            std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
        accumulated_s_.push_back(end_segment_s + distance);
        distance += end_segment_s;
        fx = nx;
        fy = ny;
    }

    // Get finite difference approximated first derivative of y and x respective
    // to s for kappa calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xds = 0.0;
        double yds = 0.0;
        if (i == 0) {
            xds = (xy_points_[i + 1].first - xy_points_[i].first) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
            yds = (xy_points_[i + 1].second - xy_points_[i].second) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
        } else if (i == points_size - 1) {
            xds = (xy_points_[i].first - xy_points_[i - 1].first) /
                  (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
            yds = (xy_points_[i].second - xy_points_[i - 1].second) /
                  (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
        } else {
            xds = (xy_points_[i + 1].first - xy_points_[i - 1].first) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i - 1));
            yds = (xy_points_[i + 1].second - xy_points_[i - 1].second) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i - 1));
        }
        x_over_s_first_derivatives.push_back(xds);
        y_over_s_first_derivatives.push_back(yds);
    }

    // Get finite difference approximated second derivative of y and x respective
    // to s for kappa calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double xdds = 0.0;
        double ydds = 0.0;
        if (i == 0) {
            xdds =
                (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
                (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
            ydds =
                (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
                (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
        } else if (i == points_size - 1) {
            xdds =
                (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
                (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
            ydds =
                (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
                (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
        } else {
            xdds = (x_over_s_first_derivatives[i + 1] -
                    x_over_s_first_derivatives[i - 1]) /
                   (accumulated_s_.at(i + 1) - accumulated_s_.at(i - 1));
            ydds = (y_over_s_first_derivatives[i + 1] -
                    y_over_s_first_derivatives[i - 1]) /
                   (accumulated_s_.at(i + 1) - accumulated_s_.at(i - 1));
        }
        x_over_s_second_derivatives.push_back(xdds);
        y_over_s_second_derivatives.push_back(ydds);
    }

    for (std::size_t i = 0; i < points_size; ++i) {
        double xds = x_over_s_first_derivatives[i];
        double yds = y_over_s_first_derivatives[i];
        double xdds = x_over_s_second_derivatives[i];
        double ydds = y_over_s_second_derivatives[i];
        double kappa =
            (xds * ydds - yds * xdds) /
            (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        kappas_.push_back(kappa);
    }

    // Dkappa calculation
    for (std::size_t i = 0; i < points_size; ++i) {
        double dkappa = 0.0;
        if (i == 0) {
            dkappa = (kappas_.at(i + 1) - kappas_.at(i)) /
                     (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
        } else if (i == points_size - 1) {
            dkappa = (kappas_.at(i) - kappas_.at(i - 1)) /
                     (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
        } else {
            dkappa = (kappas_.at(i + 1) - kappas_.at(i - 1)) /
                     (accumulated_s_.at(i + 1) - accumulated_s_.at(i - 1));
        }
        dkappas_.push_back(dkappa);
    }
    return true;
}


double DiscretePoints::GetArcLengthToClosestPoint(double x, double y) {
    if (xy_points_.empty() || xy_points_.size() != accumulated_s_.size()) return -1;

    // Lambda 表达式进行插值
    auto interpolatePoint = [&](double index) -> std::pair<double, double> {
        if (index < 0) return xy_points_.front();
        if (index > (int)xy_points_.size() - 1) return xy_points_.back();

        auto lower_index = static_cast<std::size_t>(std::floor(index));
        std::size_t upper_index = lower_index + 1;
        if (upper_index >= xy_points_.size()) upper_index = lower_index;

        double t = index - lower_index;
        double x = xy_points_[lower_index].first + t * (xy_points_[upper_index].first - xy_points_[lower_index].first);
        double y = xy_points_[lower_index].second + t * (xy_points_[upper_index].second - xy_points_[lower_index].second);

        return std::make_pair(x, y);
    };

    // Lambda 表达式计算距离
    auto distanceFunction = [&interpolatePoint, x, y](double index) -> double {
        auto point = interpolatePoint(index);
        return std::sqrt(std::pow(point.first - x, 2) + std::pow(point.second - y, 2));
    };

    // 使用 brent_find_minima 寻找最近点的索引
    std::pair<double, double> result = boost::math::tools::brent_find_minima(
        distanceFunction,
        0.0,                                         // min
        static_cast<double>(xy_points_.size() - 1),  // max
        std::numeric_limits<double>::digits);        // bits

    double closest_point_index = result.first;

    // 确保索引在有效范围内
    closest_point_index = std::max(0.0, std::min(closest_point_index, static_cast<double>(xy_points_.size() - 1)));

    // 在 accumulated_s 中插值以找到实际弧长
    double lower_index = std::floor(closest_point_index);
    double upper_index = std::ceil(closest_point_index);

    if (lower_index == upper_index) {
        return accumulated_s_[lower_index];
    }

    double lower_s = accumulated_s_[lower_index];
    double upper_s = accumulated_s_[upper_index];

    // 线性插值以计算实际弧长
    double interpolated_s = lower_s + (closest_point_index - lower_index) * (upper_s - lower_s);

    return interpolated_s;
}

double DiscretePoints::FindClosestPointIndex(double x, double y) {
    // Lambda 表达式进行插值
    auto interpolatePoint = [this](double index) -> std::pair<double, double> {
        if (index < 0) return xy_points_.front();
        if (index >= static_cast<double>(xy_points_.size())) return xy_points_.back();

        std::size_t lower_index = static_cast<std::size_t>(std::floor(index));
        std::size_t upper_index = lower_index + 1;
        if (upper_index >= xy_points_.size()) upper_index = lower_index;

        double t = index - lower_index;
        double x_interpolated = xy_points_[lower_index].first + t * (xy_points_[upper_index].first - xy_points_[lower_index].first);
        double y_interpolated = xy_points_[lower_index].second + t * (xy_points_[upper_index].second - xy_points_[lower_index].second);

        return std::make_pair(x_interpolated, y_interpolated);
    };

    // Lambda 表达式计算距离
    auto distanceFunction = [this, &interpolatePoint, x, y](double index) -> double {
        auto point = interpolatePoint(index);
        return std::sqrt(std::pow(point.first - x, 2) + std::pow(point.second - y, 2));
    };

    // 使用 brent_find_minima 寻找最近点的索引
    std::pair<double, double> result = boost::math::tools::brent_find_minima(
        distanceFunction,
        0.0,                                         // min
        static_cast<double>(xy_points_.size() - 1),  // max
        std::numeric_limits<double>::digits);        // bits

    return result.first;
}

std::pair<double, double> DiscretePoints::GetClosestPoint(double x, double y) {
    double closest_point_index = FindClosestPointIndex(x, y);
    // 插值找到最佳匹配点的坐标
    double lower_index = std::floor(closest_point_index);
    double upper_index = std::ceil(closest_point_index);
    if (upper_index >= xy_points_.size()) upper_index = lower_index;

    double t = closest_point_index - lower_index;
    double x_interpolated = xy_points_[lower_index].first + t * (xy_points_[upper_index].first - xy_points_[lower_index].first);
    double y_interpolated = xy_points_[lower_index].second + t * (xy_points_[upper_index].second - xy_points_[lower_index].second);

    return std::make_pair(x_interpolated, y_interpolated);
}

double DiscretePoints::GetHeadingAtClosestPoint(double x, double y) {
    double closest_point_index = FindClosestPointIndex(x, y);
    // 插值找到最佳匹配点的切线方向
    double lower_index = std::floor(closest_point_index);
    double upper_index = std::ceil(closest_point_index);
    if (upper_index >= headings_.size()) upper_index = lower_index;

    double t = closest_point_index - lower_index;
    double heading_interpolated = headings_[lower_index] + t * (headings_[upper_index] - headings_[lower_index]);

    return heading_interpolated;
}



