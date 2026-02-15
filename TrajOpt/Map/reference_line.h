#pragma once
#include <vector>

#include "Common/common.h"
#include "curve.h"

class DiscretizedBezierCurve {
public:
    DiscretizedBezierCurve(ReferenceBezierCurve<3>& bezierCurve, double interval) {
        double totalLength = bezierCurve.calArcLength(1.0);
        double t = 0.0, lastLength = 0.0;

        double dt = std::max(0.01, 1.0 / totalLength);  // 以曲线长度为基础的初始 dt

        while (t <= 1.0) {
            double length = bezierCurve.calArcLength(t);
            if (length - lastLength >= interval) {
                discretizedPoints.push_back(bezierCurve.evaluate(t));
                lastLength = length;
                dt = std::min(dt * 1.1, 0.1);  // 如果间隔适宜，可以尝试增加 dt
            } else {
                dt = std::max(dt * 0.9, 0.001);  // 如果间隔太小，减少 dt
            }
            t += dt;
        }
    }
    const std::vector<std::vector<double>>& getDiscretizedPoints() const {
        return discretizedPoints;
    }

    // ... 之前的定义 ...

    // 计算给定弧长 s 处的切线角度
    double tangentAngleAtArcLength(double s) {
        int index = findIndexByArcLength(s);
        if (index < 0 || index >= discretizedPoints.size() - 1)
            return 0;  // 或适当处理边界情况

        auto& p1 = discretizedPoints[index];
        auto& p2 = discretizedPoints[index + 1];
        return std::atan2(p2[1] - p1[1], p2[0] - p1[0]);
    }

    // 计算给定弧长 s 处的曲率 kappa
    double kappaAtArcLength(double s) {
        int index = findIndexByArcLength(s);
        if (index <= 0 || index >= discretizedPoints.size() - 1)
            return 0;  // 或适当处理边界情况

        auto& p0 = discretizedPoints[index - 1];
        auto& p1 = discretizedPoints[index];
        auto& p2 = discretizedPoints[index + 1];

        double dx1 = p1[0] - p0[0];
        double dy1 = p1[1] - p0[1];
        double dx2 = p2[0] - p1[0];
        double dy2 = p2[1] - p1[1];

        double num = abs(dx1 * dy2 - dy1 * dx2);
        double den = std::pow(dx1 * dx1 + dy1 * dy1, 1.5);

        return num / den;
    }

    // 根据弧长找到最接近的点的索引
    int findIndexByArcLength(double s) {
        double length = 0.0;
        for (int i = 0; i < discretizedPoints.size() - 1; ++i) {
            double segmentLength = distance(discretizedPoints[i], discretizedPoints[i + 1]);
            if (length + segmentLength > s)
                return i;
            length += segmentLength;
        }
        return (int)discretizedPoints.size() - 1;
    }

    // 计算两点间的距离
    double distance(const std::vector<double>& p1, const std::vector<double>& p2) {
        return std::sqrt(std::pow(p2[0] - p1[0], 2) + std::pow(p2[1] - p1[1], 2));
    }

private:
    std::vector<std::vector<double>> discretizedPoints;
};
