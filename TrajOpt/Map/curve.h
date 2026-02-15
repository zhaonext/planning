#pragma once
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <vector>

#include "Common/common.h"
#include "PathSegment/vec2d.h"
#include "boost/math/quadrature/gauss.hpp"
#include "boost/math/quadrature/trapezoidal.hpp"
#include "boost/math/tools/minima.hpp"

template <int N>
class ReferenceBezierCurve {
public:
    explicit ReferenceBezierCurve(std::vector<std::vector<double>> control_points) : degree(N) {
        controlPoints.resize(N + 1, std::vector<double>(2, 0.0));
        setControlPoints(control_points);
        precomputeBinomCoeffs();
        precomputeArcLengths();
    }

    void setControlPoints(const std::vector<std::vector<double>> &points) {
        if (points.size() == degree + 1 && points[0].size() == 2) {
            controlPoints = points;
        } else {
            AERROR << "Error: The number of control points must be " << (degree + 1)
                   << ", and each point must contain 2 coordinates. ";
        }
    }

    std::vector<double> evaluate(double t) {
        if (t < 0.0 || t > 1.0) {
            AERROR << "Error: evaluate parameter value must be between 0 and 1. Given t = " << t;
            return {0.0, 0.0};
        }

        std::vector<double> point = {0.0, 0.0};
        for (int i = 0; i <= degree; ++i) {
            double blend = binomialCoefficient(degree, i) * std::pow(t, i) * std::pow(1 - t, degree - i);
            point[0] += controlPoints[i][0] * blend;
            point[1] += controlPoints[i][1] * blend;
        }
        return point;
    }
    std::vector<double> evaluateFirstDerivative(double t) {
        if (t < 0.0 || t > 1.0) {
            AERROR << "Error: evaluateFirstDerivative parameter value must be between 0 and 1. Given t = " << t;
        }

        std::vector<double> derivative = {0.0, 0.0};
        for (int i = 0; i < degree; ++i) {
            double blendX = degree * (controlPoints[i + 1][0] - controlPoints[i][0]);
            double blendY = degree * (controlPoints[i + 1][1] - controlPoints[i][1]);

            double binCoeff = binomialCoefficient(degree - 1, i);
            double tPower = std::pow(t, i);
            double oneMinusTPower = std::pow(1 - t, degree - 1 - i);

            derivative[0] += blendX * binCoeff * tPower * oneMinusTPower;
            derivative[1] += blendY * binCoeff * tPower * oneMinusTPower;
        }
        return derivative;
    }
    std::vector<double> evaluateSecondDerivative(double t) {
        if (t < 0.0 || t > 1.0) {
            AERROR << "Error: evaluateSecondDerivative parameter value must be between 0 and 1. Given t = " << t;
        }

        const double coeff = degree * (degree - 1);
        double tPower = 1.0;
        double oneMinusTPower = std::pow(1 - t, degree - 2);

        std::vector<double> derivative = {0.0, 0.0};
        for (int i = 0; i < degree - 1; ++i) {
            double blendX = coeff * (controlPoints[i + 2][0] - 2 * controlPoints[i + 1][0] + controlPoints[i][0]);
            double blendY = coeff * (controlPoints[i + 2][1] - 2 * controlPoints[i + 1][1] + controlPoints[i][1]);

            double binomCoeff = binomialCoefficient(degree - 2, i);
            derivative[0] += blendX * binomCoeff * tPower * oneMinusTPower;
            derivative[1] += blendY * binomCoeff * tPower * oneMinusTPower;

            tPower *= t;
            oneMinusTPower /= (1 - t);
        }
        return derivative;
    }
    std::vector<double> evaluateThirdDerivative(double t) {
        if (t < 0.0 || t > 1.0) {
            AERROR << "Error: evaluateThirdDerivative parameter value must be between 0 and 1. Given t = " << t;
        }

        const double coeff = degree * (degree - 1) * (degree - 2);
        double tPower = 1.0;
        double oneMinusTPower = std::pow(1 - t, degree - 3);

        std::vector<double> derivative = {0.0, 0.0};
        for (int i = 0; i < degree - 2; ++i) {
            double blendX = coeff * (controlPoints[i + 3][0] - 3 * controlPoints[i + 2][0] + 3 * controlPoints[i + 1][0] - controlPoints[i][0]);
            double blendY = coeff * (controlPoints[i + 3][1] - 3 * controlPoints[i + 2][1] + 3 * controlPoints[i + 1][1] - controlPoints[i][1]);

            double binomCoeff = binomialCoefficient(degree - 3, i);
            derivative[0] += blendX * binomCoeff * tPower * oneMinusTPower;
            derivative[1] += blendY * binomCoeff * tPower * oneMinusTPower;

            tPower *= t;
            oneMinusTPower /= (1 - t);
        }
        return derivative;
    }

    double calTangentAngle(double t) {
        std::vector<double> firstDerivative = evaluateFirstDerivative(t);
        double x = firstDerivative[0];
        double y = firstDerivative[1];
        if (x == 0) {
            if (y > 0) {
                return std::numeric_limits<double>::infinity();  // 或者返回 M_PI/2
            } else if (y < 0) {
                return -std::numeric_limits<double>::infinity();  // 或者返回 -M_PI/2
            } else {
                return std::nan("1");  // 当 x 和 y 都为 0 时
            }
        } else if (y == 0) {
            return 0;
        } else {
            double tangentAngle = std::atan2(y, x);
            return tangentAngle;
        }
    }
    double calKappaByT(double t) {
        const std::vector<double> &firstDerivative = evaluateFirstDerivative(t);
        const std::vector<double> &secondDerivative = evaluateSecondDerivative(t);

        // 计算两个导数的点积，并检查是否为零，避免不必要的后续计算。
        double dotProduct = firstDerivative[0] * firstDerivative[0] + firstDerivative[1] * firstDerivative[1];
        if (dotProduct == 0.0) {
            return 0.0;  // 避免除以零的情况。
        }

        double numerator = firstDerivative[0] * secondDerivative[1] - firstDerivative[1] * secondDerivative[0];
        if (numerator == 0.0) {
            return 0.0;  // 如果分子为零，则整个表达式为零。
        }

        double denominator = std::pow(dotProduct, 1.5);
        return numerator / denominator;
    }
    double calDKappaByT(double t) {
        std::vector<double> firstDerivative = evaluateFirstDerivative(t);
        std::vector<double> secondDerivative = evaluateSecondDerivative(t);
        std::vector<double> thirdDerivative = evaluateThirdDerivative(t);

        double val1 = firstDerivative[0] * thirdDerivative[1] - firstDerivative[1] * thirdDerivative[0];
        double val2 = firstDerivative[0] * firstDerivative[0] + firstDerivative[1] * firstDerivative[1];

        double val3 = firstDerivative[0] * secondDerivative[1] - firstDerivative[1] * secondDerivative[0];
        double val4 = firstDerivative[0] * secondDerivative[0] + firstDerivative[1] * secondDerivative[1];

        double dkappa_res = (val1 * val2 - 3 * val3 * val4) / std::pow(val2, 2.5);
        return dkappa_res;
    }

    void precomputeArcLengths(int tableSize = 500) {
        arcLengthTable.resize(tableSize);
        double accumulatedArcLength = 0.0;
        double dt = 1.0 / (tableSize - 1);
        arcLengthTable[0] = 0.0;

        for (int i = 1; i < arcLengthTable.size(); ++i) {
            double t1 = (i - 1) * dt;
            double t2 = i * dt;
            std::vector<double> d1 = evaluateFirstDerivative(t1);
            std::vector<double> d2 = evaluateFirstDerivative(t2);
            double len1 = std::sqrt(d1[0] * d1[0] + d1[1] * d1[1]);
            double len2 = std::sqrt(d2[0] * d2[0] + d2[1] * d2[1]);
            double segmentLength = (len1 + len2) * dt / 2.0;

            accumulatedArcLength += segmentLength;
            arcLengthTable[i] = accumulatedArcLength;
        }
    }

    double findTByArcLengthTrapezoidal(double s) {
        if (arcLengthTable.empty()) {
            AERROR << "Error: arcLengthTable is empty.";
            return 0.0;  // Return a distinct error value
        }

        if (s < 0.0) {
            AERROR << "Error: s cannot be negative.";
            return -1.0;  // Return a distinct error value
        }

        if (s >= arcLengthTable.back()) {
            AERROR << "Warning: s is beyond the range of arcLengthTable.";
            return 1.0;
        }

        const int idx = std::lower_bound(arcLengthTable.begin(), arcLengthTable.end(), s) - arcLengthTable.begin();

        if (idx == 0) return 0.0;

        double s1 = arcLengthTable[idx - 1];
        double s2 = arcLengthTable[idx];
        double dt = 1.0 / (arcLengthTable.size() - 1);
        double alpha = (s - s1) / (s2 - s1);
        return (idx - 1 + alpha) * dt;
    }

    double calArcLength(double t) {
        auto integrand = [&](double t) -> double {
            std::vector<double> d = evaluateFirstDerivative(t);
            return std::sqrt(d[0] * d[0] + d[1] * d[1]);
        };
        double error;
        double s = boost::math::quadrature::trapezoidal([&](double t) { return integrand(t); }, 0.0, t, error);
        //    double s = boost::math::quadrature::gauss<double, 15>::integrate([&](double t) { return integrand(t); }, 0.0, t);

        return s;
    }
    double calKappaByS(double s) {
        double t = findTByArcLengthTrapezoidal(s);
        return calKappaByT(t);
    }
    double calDKappaByS(double s) {
        double t = findTByArcLengthTrapezoidal(s);
        return calDKappaByT(t);
    }

    double minDisToBezierIndexT(const double x, const double y) {
        std::vector<double> Q{x, y};

        auto distanceToBezier = [&](double t) -> double {
            std::vector<double> B = evaluate(t);
            return DistanceSquared(B, Q);
        };

        double lowerBound = 0.0;
        double upperBound = 1.0;
        int bits = std::numeric_limits<double>::digits;  // 精度
        std::pair<double, double> result =
            boost::math::tools::brent_find_minima(distanceToBezier, lowerBound, upperBound, bits);
        return result.first;  // 返回t值
    }

    void print_controlPoints() {
        int index = 0;
        for (const auto &point : controlPoints) {
            AINFO << "control_point" << index << " :(" << point.at(0) << ", " << point.at(1) << ")";
            ++index;
        }
    }

    State calInitState(const State &X) {
        Eigen::Matrix<double, NUM_STATE, 1> result;

        // 计算 t 值
        double x_ego = X.x, y_ego = X.y, v_ego = X.v, phi_ego = X.phi;
        double t = minDisToBezierIndexT(x_ego, y_ego);
        // 计算 s 值
        double s = calArcLength(t);

        result(0) = s;

        // 计算 en 向量
        std::vector<double> d = evaluateFirstDerivative(t);
        Eigen::Vector2d tangentDirection(d.at(0), d.at(1));
        tangentDirection.normalize();
        Eigen::Matrix2d rotation90;
        rotation90 << 0, -1, 1, 0;
        Eigen::Vector2d en = rotation90 * tangentDirection;
        en.normalize();

        // 计算 n 值
        auto ref_point = evaluate(t);
        double r_x = ref_point.at(0);
        double r_y = ref_point.at(1);
        Eigen::Vector2d err{x_ego - r_x, y_ego - r_y};
        double n = err.transpose() * en;
        result(1) = n;

        // 计算 alpha 值
        double ref_phi = calTangentAngle(t);
        //    double alpha = phi_ego - M_PI + ref_phi;
        //    double alpha = phi_ego + std::abs(M_PI + ref_phi);
        double alpha = phi_ego - ref_phi;
        result(2) = alpha;

        // 自车在笛卡尔坐标系下的状态
        result(3) = x_ego;
        result(4) = y_ego;
        result(5) = v_ego;
        result(6) = phi_ego;

        auto state = vectorToState(result);
        return state;
    }
    std::vector<ObstacleInfo> sampleObstaclesAlongBezierCurve(int sampleCount, unsigned int seed = 0) {
        std::vector<ObstacleInfo> obstacles;

        // 设置随机数生成器和分布
        std::mt19937 gen(seed);  // 使用固定种子初始化生成器
        std::uniform_real_distribution<> y_offset_dist(-LANE_WIDTH + VEHICLE_WIDTH * 1.5, LANE_WIDTH - VEHICLE_WIDTH * 1.5);
        std::uniform_real_distribution<> speed_dist(LANE_SPEED_LIMIT / 3.6 * 0.4, LANE_SPEED_LIMIT / 3.6);  // 转换为 m/s

        const double startFraction = SAMPLE_START_SECTION;  // 曲线的起始部分
        const double endFraction = SAMPLE_END_SECTION;      // 曲线的结束部分
        double previous_tangent = calTangentAngle(0);
        for (int i = 0; i < sampleCount; ++i) {
            const double t = startFraction + (endFraction - startFraction) * static_cast<double>(i) / (sampleCount - 1);

            std::vector<double> point = evaluate(t);
            const double current_tangent = calTangentAngle(t);

            // 计算航向角差异
            double angle_diff = current_tangent - previous_tangent;

            // 标准化角度差，确保在 -π 到 π 范围内
            angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

            // 假设采样间隔是固定的，例如每个采样点间隔1秒
            double time_interval = 1.0;  // 可以根据实际情况调整
            double psi_dot = angle_diff / time_interval;

            // 计算航向角
            const double phi = current_tangent;

            // 创建障碍物信息
            ObstacleInfo obs;
            obs.x = point[0] + y_offset_dist(gen) * std::cos(phi + M_PI / 2);  // 偏移应垂直于切线
            obs.y = point[1] + y_offset_dist(gen) * std::sin(phi + M_PI / 2);  // 偏移应垂直于切线
            obs.v = speed_dist(gen);
            obs.psi_dot = psi_dot;

            // TODO :: 反向行驶

            //            if (obs.y > 0) {
            //                obs.phi = phi;  // 正向行驶，航向角与曲线切线相同
            //            } else {
            //                obs.phi = phi + M_PI;  // 反向行驶，航向角与曲线切线相反
            //            }

            obs.phi = phi;

            obstacles.push_back(obs);
        }

        return obstacles;
    }

    std::vector<Vec2d> discretizePointsToVec2d(double fixedArcLength) {
        double totalLength = calArcLength(1.0);                   // 计算曲线的总弧长
        int numPoints = std::ceil(totalLength / fixedArcLength);  // 确定离散化点的数量

        std::vector<Vec2d> discretePoints;
        for (int i = 0; i < numPoints; ++i) {
            double s = i * fixedArcLength;  // 计算当前点的弧长
            if (s > totalLength) {
                s = totalLength;  // 确保不超过曲线的总弧长
            }
            double t = findTByArcLengthTrapezoidal(s);  // 找到对应的 t 值
            double x = evaluate(t).at(0);
            double y = evaluate(t).at(1);
            Vec2d point(x, y);  // 计算曲线上的点
            discretePoints.push_back(point);
        }

        return discretePoints;
    }

    std::vector<std::pair<double, double>> discretizePointsToPair(double fixedArcLength) {
        double totalLength = calArcLength(1.0);                   // 计算曲线的总弧长
        int numPoints = std::ceil(totalLength / fixedArcLength);  // 确定离散化点的数量

        std::vector<std::pair<double, double>> discretePoints;
        for (int i = 0; i < numPoints; ++i) {
            double s = i * fixedArcLength;  // 计算当前点的弧长
            if (s > totalLength) {
                s = totalLength;  // 确保不超过曲线的总弧长
            }
            double t = findTByArcLengthTrapezoidal(s);                              // 找到对应的 t 值
            std::pair<double, double> point(evaluate(t).at(0), evaluate(t).at(1));  // 计算曲线上的点
            discretePoints.push_back(point);
        }

        return discretePoints;
    }

private:
    int degree;
    std::vector<std::vector<double>> controlPoints;
    std::vector<double> arcLengthTable;  // 存储弧长值
private:
    double DistanceSquared(const std::vector<double> &p1, const std::vector<double> &p2) {
        double dx = p1[0] - p2[0];
        double dy = p1[1] - p2[1];
        return dx * dx + dy * dy;
    }

    // 查表法
    static constexpr int MAX_DEGREE = 10;
    static std::vector<std::vector<int>> binomCoeffs;

    void precomputeBinomCoeffs() {
        for (int n = 0; n <= MAX_DEGREE; ++n) {
            for (int k = 0; k <= n; ++k) {
                if (k == 0 || k == n) {
                    binomCoeffs[n][k] = 1;
                } else {
                    binomCoeffs[n][k] = binomCoeffs[n - 1][k - 1] + binomCoeffs[n - 1][k];
                }
            }
        }
    }

    int binomialCoefficient(int n, int k) {
        if (n < 0 || n > MAX_DEGREE || k < 0 || k > n) {
            return 0;  // or throw an exception
        }
        return binomCoeffs[n][k];
    }
};

template <int N>
std::vector<std::vector<int>> ReferenceBezierCurve<N>::binomCoeffs(MAX_DEGREE + 1, std::vector<int>(MAX_DEGREE + 1, 0));