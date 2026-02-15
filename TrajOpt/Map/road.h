#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "Common/log.h"
#include "curve.h"
using LaneDataMap = std::unordered_map<std::string, std::pair<std::vector<double>, std::vector<double>>>;

class Lane {
public:
    Lane() = default;
    Lane(int id, const std::shared_ptr<ReferenceBezierCurve<BEZIER_ORDER>>& refLine, double offset, int horizon)
        : laneID(id), referenceLine(refLine) {
        generateLaneData(offset, horizon);
    }

    int getLaneID() const { return laneID; }
    LaneDataMap getLaneData() const { return laneData; }
    const std::shared_ptr<ReferenceBezierCurve<BEZIER_ORDER>>& getReferenceLine() const { return referenceLine; }
    Vec2d getLaneEndPoints() const { return Vec2d{referenceLine->evaluate(1.0).at(0), referenceLine->evaluate(1.0).at(1)}; }

private:
    int laneID{};          // 车道ID
    LaneDataMap laneData;  // 存储车道数据
    std::shared_ptr<ReferenceBezierCurve<BEZIER_ORDER>> referenceLine;

    void generateLaneData(double offset, int horizon) {
        const double step = 1.0 / horizon;
        std::vector<double> x_ref, y_ref;
        std::vector<double> x_left, y_left, x_right, y_right;

        for (int i = 0; i <= horizon; ++i) {
            const double t = i * step;
            std::vector<double> point = referenceLine->evaluate(t);
            const double slope = referenceLine->calTangentAngle(t);

            x_ref.push_back(point[0]);
            y_ref.push_back(point[1]);

            double dx, dy;
            if (std::isinf(slope)) {
                dx = offset;
                dy = 0;
            } else if (slope == 0) {
                dx = 0;
                dy = -offset;
            } else {
                const double normal_slope = -1.0 / slope;
                dx = offset / sqrt(1 + normal_slope * normal_slope);
                dy = normal_slope * dx;
            }

            x_left.push_back(point[0] - dx);
            y_left.push_back(point[1] - dy);
            x_right.push_back(point[0] + dx);
            y_right.push_back(point[1] + dy);
        }

        laneData["reference_line"] = std::make_pair(x_ref, y_ref);
        laneData["left_boundary"] = std::make_pair(x_left, y_left);
        laneData["right_boundary"] = std::make_pair(x_right, y_right);
    }
};

class Road {
public:
    Road() = default;
    explicit Road(int id) : road_id_(id) {}
    Road(int id, const std::unordered_map<int, std::vector<std::vector<double>>>& baseControlPoints, double offset, int horizon) : road_id_(id) {
        for (const auto& pair : baseControlPoints) {
            auto [lane_id, controlPoints] = pair;
            auto ref_line = std::make_shared<ReferenceBezierCurve<BEZIER_ORDER>>(controlPoints);
            lanes.emplace_back(lane_id, ref_line, offset, horizon);
        }
    }
    Road(int id, const std::vector<std::vector<std::vector<double>>>& baseControlPoints, double offset, int horizon) : road_id_(id) {
        for (int i = 0; i < baseControlPoints.size(); ++i) {
            auto ref_line = std::make_shared<ReferenceBezierCurve<BEZIER_ORDER>>(baseControlPoints.at(i));
            lanes.emplace_back(i + 1, ref_line, offset, horizon);
        }
    }
    Road(const std::vector<std::vector<std::vector<double>>>& baseControlPoints, double offset, int horizon) {
        for (int i = 0; i < baseControlPoints.size(); ++i) {
            auto refLine = std::make_shared<ReferenceBezierCurve<BEZIER_ORDER>>(baseControlPoints.at(i));
            lanes.emplace_back(i, refLine, offset, horizon);
        }
    }

    const LaneDataMap getLaneDataForLaneID(int laneID) const {
        for (const auto& lane : lanes) {
            if (lane.getLaneID() == laneID) {
                return lane.getLaneData();
            }
        }
        AERROR << "Lane ID not found\n";
        static LaneDataMap emptyMap;
        return emptyMap;  // 返回空映射
    }

    const std::vector<Lane>& getLanes() const { return lanes; }
    int getId() const { return road_id_; }

    Lane getLaneByID(int laneID) const {
        for (const auto& lane : lanes) {
            if (lane.getLaneID() == laneID) {
                return lane;
            }
        }

        AERROR << "Lane ID " << laneID << " not found in road " << road_id_;
        return {};
    }

    Vec2d getLandEndPoint(int lane_id) { return getLaneByID(lane_id).getLaneEndPoints(); }

private:
    std::vector<Lane> lanes;
    int road_id_{};
};
