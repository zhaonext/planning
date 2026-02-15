#pragma once
#include <vector>

#include "Map/road.h"
#include "PathSegment/path.h"

class Routing {
public:
    explicit Routing(const std::vector<Lane>& routing_lane, double discrete_interval = 0.25) : routing_lane_(routing_lane), discrete_interval_(discrete_interval) { getReferenceLinePoints(); }

    std::shared_ptr<Path> getReferenceLine(const Vec2d& point) {
        if (!ref_path_) {
            AERROR << "Error: ref_path_ is null.";
            return nullptr;
        }

        int index = ref_path_->GetNearestPoint(point);
        int points_behind = static_cast<int>(20.0 / discrete_interval_);
        int points_ahead = static_cast<int>(180.0 / discrete_interval_);

        int start = std::max(0, index - points_behind);
        int end = std::min(static_cast<int>(discretize_points_.size()) - 1, index + points_ahead);

        if (end < start || discretize_points_.empty()) {
            AERROR << "Error: Invalid start or end index.";
            return nullptr;
        }

        std::vector<Vec2d> result;
        result.insert(result.end(), discretize_points_.begin() + start, discretize_points_.begin() + end + 1);

        std::shared_ptr<Path> new_ref_path = std::make_shared<Path>(result);
        return new_ref_path;
    }
    const shared_ptr<Path>& getRoutingPath() const { return ref_path_; }

private:
    void getReferenceLinePoints() {
        for (const auto& lane : routing_lane_) {
            auto lane_dis_points = lane.getReferenceLine()->discretizePointsToVec2d(discrete_interval_);
            discretize_points_.insert(discretize_points_.end(), lane_dis_points.begin(), lane_dis_points.end());
        }
        ref_path_ = std::make_shared<Path>(discretize_points_);
    }

private:
    std::vector<Lane> routing_lane_;
    double discrete_interval_;  // m

    std::vector<Vec2d> discretize_points_;
    std::shared_ptr<Path> ref_path_;

public:
};