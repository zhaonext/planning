#include "path.h"

#include <algorithm>
#include <limits>
#include <unordered_map>

#include "line_segment2d.h"
#include "math_utils.h"
#include "polygon2d.h"
#include "vec2d.h"

// it should be class member of path, but is can be changed during check
int last_project_index_ = 0;

// https://nacto.org/publication/urban-street-design-guide/street-design-elements/lane-width/
// DEFINE_double(default_lane_width, 3.048, "default lane width is about 10
// feet");

using std::placeholders::_1;

const double kSampleDistance = 1.0;

bool Path::Init() {
    if (!InitPoints()) return false;
    if (!InitPointIndex()) return false;
    return true;
}

void Path::CreateKDTree() {
    AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 16;

    segment_box_list_.clear();
    for (size_t id = 0; id < segments_.size(); ++id) {
        const auto &segment = segments_[id];
        segment_box_list_.emplace_back(
            AABox2d(segment.start(), segment.end()), (void *)nullptr,
            &segment, id);
    }
    path_segment_kdtree_.reset(new PathSegmentKDTree(segment_box_list_, params));
}

bool Path::InitPoints() {
    num_points_ = path_points_.size();
    if (!CHECK_GE(num_points_, 2UL)) {
        return false;
    }

    accumulated_s_.clear();
    accumulated_s_.reserve(num_points_);
    segments_.clear();
    segments_.reserve(num_points_);
    unit_directions_.clear();
    unit_directions_.reserve(num_points_);
    double s = 0.0;
    for (size_t i = 0; i < num_points_; ++i) {
        accumulated_s_.push_back(s);
        Vec2d heading;
        if (i + 1UL >= num_points_) {
            heading = path_points_[i] - path_points_[i - 1];
        } else {
            segments_.emplace_back(path_points_[i], path_points_[i + 1]);  // MapPathPoint两两生成一个LaneSegment2d
            heading = path_points_[i + 1] - path_points_[i];               // LaneSegment2d的方向，段终点-段起点
            // TODO(All): use heading.length when all adjacent lanes are guarantee to
            // be connected.
            s += heading.Length();
        }
        heading.Normalize();  // 方向正则，长度归一
        unit_directions_.push_back(heading);
    }
    length_ = s;
    num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
    num_segments_ = num_points_ - 1;

    if (!CHECK_EQ(accumulated_s_.size(), num_points_) ||
        !CHECK_EQ(unit_directions_.size(), num_points_) ||
        !CHECK_EQ(segments_.size(), num_segments_)) {
        return false;
    }

    return true;
}

//  Path::InitPointIndex函数计算每个采样点的上界MapPathPoint
bool Path::InitPointIndex() {
    last_point_index_.clear();
    last_point_index_.reserve(num_sample_points_);
    double s = 0.0;
    int last_index = 0;
    for (size_t i = 0; i < num_sample_points_; ++i) {
        // 向后遍历，得到采样点(对应累积距离为s)的上界MapPathPoint
        while (last_index + 1UL < num_points_ &&
               accumulated_s_[last_index + 1] <= s) {
            ++last_index;
        }
        last_point_index_.push_back(last_index);
        s += kSampleDistance;  // 下一个采样点的累积距离，加上kSampleDistance
    }
    if (!CHECK_EQ(last_point_index_.size(), num_sample_points_)) {
        return false;
    }

    return true;
}

double Path::GetSFromIndex(const InterpolatedIndex &index) const {
    if (index.id < 0) {
        return 0.0;
    }
    if (index.id >= static_cast<int>(num_points_)) {
        return length_;
    }
    return accumulated_s_[index.id] + index.offset;
}

InterpolatedIndex Path::GetIndexFromS(double s) const {
    if (s <= 0.0) {
        return {0, 0.0};
    }
    if (!CHECK_GT(num_points_, 0UL)) {
        return {0, 0.0};
    }
    if (s >= length_) {
        return {static_cast<int>(num_points_) - 1, 0.0};
    }
    const auto sample_id = static_cast<size_t>(s / kSampleDistance);  // Step 1. 可以计算大致的采样点坐标
    if (sample_id >= num_sample_points_) {
        return {static_cast<int>(num_points_) - 1, 0.0};
    }
    const auto next_sample_id = sample_id + 1;
    if (last_point_index_.size() <= sample_id) {
        return {0, 0.0};
    }
    size_t low = last_point_index_[sample_id];  // Step 2. 计算理论下界
    // Step 3. 计算理论上界
    size_t high = (next_sample_id < num_sample_points_
                       ? std::min(num_points_, last_point_index_[next_sample_id] + 1UL)
                       : num_points_);
    // Step 4. 二分法计算真实下界以及与下届的累计距离差
    while (low + 1 < high) {
        const auto mid = (low + high) / 2;
        if (accumulated_s_[mid] <= s) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return {static_cast<int>(low), s - accumulated_s_[low]};
}

int Path::GetNearestPoint(const Vec2d &point) const {
    double s = 0.0;
    double l = 0.0;
    GetNearestPoint(point, &s, &l);
    return GetIndexFromS(s).id;
}

bool Path::GetNearestPoint(const Vec2d &point, double *accumulate_s,
                           double *lateral) const {
    double distance = 0.0;
    return GetNearestPoint(point, accumulate_s, lateral, &distance);
}

bool Path::GetNearestPoint(const Vec2d &point, double *accumulate_s,
                           double *lateral, double *min_distance) const {
    if (!GetProjection(point, accumulate_s, lateral, min_distance)) {
        return false;
    }
    if (*accumulate_s < 0.0) {
        *accumulate_s = 0.0;
        *min_distance = point.DistanceTo(path_points_[0]);
    } else if (*accumulate_s > length_) {
        *accumulate_s = length_;
        *min_distance = point.DistanceTo(path_points_.back());
    }
    return true;
}

bool Path::GetProjection(const Vec2d &point, double *accumulate_s,
                         double *lateral) const {
    double distance = 0.0;
    return GetProjection(point, accumulate_s, lateral, &distance);
}

bool Path::GetProjectionWithHueristicParams(const Vec2d &point,
                                            const double hueristic_start_s,
                                            const double hueristic_end_s,
                                            double *accumulate_s,
                                            double *lateral,
                                            double *min_distance) const {
    if (segments_.empty()) {
        return false;
    }
    if (accumulate_s == nullptr || lateral == nullptr ||
        min_distance == nullptr) {
        return false;
    }
    if (!CHECK_GE(num_points_, 2UL)) {
        return false;
    }
    *min_distance = std::numeric_limits<double>::infinity();

    int start_interpolation_index = GetIndexFromS(hueristic_start_s).id;
    int end_interpolation_index = static_cast<int>(
        std::fmin(num_segments_, GetIndexFromS(hueristic_end_s).id + 1));
    size_t min_index = start_interpolation_index;
    for (int i = start_interpolation_index; i < end_interpolation_index; ++i) {
        const double distance = segments_[i].DistanceSquareTo(point);
        if (distance < *min_distance) {
            min_index = i;
            *min_distance = distance;
        }
    }

    if (min_index >= segments_.size()) {
        min_index = min(min_index, segments_.size() - 1);
    }

    *min_distance = std::sqrt(*min_distance);
    const auto &nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);
    if (min_index == 0) {
        *accumulate_s = std::min(proj, nearest_seg.length());
        if (proj < 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else if (min_index == num_segments_ - 1) {
        *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
        if (proj > 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else {
        *accumulate_s = accumulated_s_[min_index] +
                        std::max(0.0, std::min(proj, nearest_seg.length()));
        *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
    return true;
}

bool Path::GetProjection(const Vec2d &point, double *accumulate_s,
                         double *lateral, double *min_distance) const {
    if (segments_.empty()) {
        return false;
    }
    if (accumulate_s == nullptr || lateral == nullptr ||
        min_distance == nullptr) {
        return false;
    }
    if (use_path_approximation_) {
        return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                            min_distance);
    }
    if (!CHECK_GE(num_points_, 2UL)) {
        return false;
    }
    *min_distance = std::numeric_limits<double>::infinity();
    size_t min_index = 0;
    if (path_segment_kdtree_) {
        auto min_seg = path_segment_kdtree_->GetNearestObject(point);
        *min_distance = min_seg->DistanceTo(point);
        min_index = min_seg->id();
    } else {
        for (size_t i = 0; i < num_segments_; ++i) {
            const double distance = segments_[i].DistanceSquareTo(point);
            if (distance < *min_distance) {
                min_index = i;
                *min_distance = distance;
            }
        }
        *min_distance = std::sqrt(*min_distance);
    }
    const auto &nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);
    if (min_index == 0) {
        *accumulate_s = std::min(proj, nearest_seg.length());
        if (proj < 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else if (min_index == num_segments_ - 1UL) {
        *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
        if (proj > 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else {
        *accumulate_s = accumulated_s_[min_index] +
                        std::max(0.0, std::min(proj, nearest_seg.length()));
        *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
    return true;
}

bool Path::GetProjectionQuick(const Vec2d &point, double *accumulate_s,
                              double *lateral, double *min_distance, bool first_search) const {
    if (segments_.empty()) {
        return false;
    }
    if (accumulate_s == nullptr || lateral == nullptr ||
        min_distance == nullptr) {
        return false;
    }
    if (use_path_approximation_) {
        return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                            min_distance);
    }
    if (!CHECK_GE(num_points_, 2UL)) {
        return false;
    }
    *min_distance = std::numeric_limits<double>::infinity();
    size_t min_index = 0;
    int first_index = 0;
    int last_index = num_segments_;
    if (!first_search) {
        first_index = std::max(first_index, last_project_index_ - 2);
        last_index = std::min(last_index, last_project_index_ + 5);
    }
    for (int i = first_index; i < last_index; ++i) {
        const double distance = segments_[i].DistanceSquareTo(point);
        if (distance < *min_distance) {
            min_index = i;
            *min_distance = distance;
        }
    }
    last_project_index_ = min_index;
    *min_distance = std::sqrt(*min_distance);
    const auto &nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);
    if (min_index == 0) {
        *accumulate_s = std::min(proj, nearest_seg.length());
        if (proj < 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else if (min_index == num_segments_ - 1UL) {
        *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
        if (proj > 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else {
        *accumulate_s = accumulated_s_[min_index] +
                        std::max(0.0, std::min(proj, nearest_seg.length()));
        *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
    return true;
}

bool Path::GetHeadingAlongPath(const Vec2d &point, double *heading) const {
    if (heading == nullptr) {
        return false;
    }
    double s = 0;
    double l = 0;
    if (GetProjection(point, &s, &l)) {
        //        *heading = GetSmoothPoint(s).heading();
        // TODO: To be optimized
        *heading = segments_[GetIndexFromS(s).id].heading();
        return true;
    }
    return false;
}
bool Path::GetHeadingAlongPath(const Vec2d &point, Vec2d *tangentDirection) const {
    if (tangentDirection == nullptr) {
        return false;
    }
    double s = 0;
    double l = 0;
    if (GetProjection(point, &s, &l)) {
        //        *heading = GetSmoothPoint(s).heading();
        // TODO: To be optimized
        *tangentDirection = segments_[GetIndexFromS(s).id].unit_direction();
        return true;
    }
    return false;
}

double PathApproximation::compute_max_error(const Path &path, const int s,
                                            const int t) {
    if (s + 1 >= t) {
        return 0.0;
    }
    const auto &points = path.path_points();
    const LineSegment2d segment(points[s], points[t]);
    double max_distance_sqr = 0.0;
    for (int i = s + 1; i < t; ++i) {
        max_distance_sqr =
            std::max(max_distance_sqr, segment.DistanceSquareTo(points[i]));
    }
    return sqrt(max_distance_sqr);
}

bool PathApproximation::is_within_max_error(const Path &path, const int s,
                                            const int t) {
    if (s + 1 >= t) {
        return true;
    }
    const auto &points = path.path_points();
    const LineSegment2d segment(points[s], points[t]);
    for (int i = s + 1; i < t; ++i) {
        if (segment.DistanceSquareTo(points[i]) > max_sqr_error_) {
            return false;
        }
    }
    return true;
}

void PathApproximation::Init(const Path &path) {
    InitDilute(path);
    InitProjections(path);
}

void PathApproximation::InitDilute(const Path &path) {
    const int num_original_points = path.num_points();
    original_ids_.clear();
    int last_idx = 0;
    while (last_idx < num_original_points - 1) {
        original_ids_.push_back(last_idx);
        int next_idx = last_idx + 1;
        int delta = 2;
        for (; last_idx + delta < num_original_points; delta *= 2) {
            if (!is_within_max_error(path, last_idx, last_idx + delta)) {
                break;
            }
            next_idx = last_idx + delta;
        }
        for (; delta > 0; delta /= 2) {
            if (next_idx + delta < num_original_points &&
                is_within_max_error(path, last_idx, next_idx + delta)) {
                next_idx += delta;
            }
        }
        last_idx = next_idx;
    }
    original_ids_.push_back(last_idx);
    num_points_ = static_cast<int>(original_ids_.size());
    if (num_points_ == 0) {
        return;
    }

    segments_.clear();
    segments_.reserve(num_points_ - 1);
    for (int i = 0; i < num_points_ - 1; ++i) {
        segments_.emplace_back(path.path_points()[original_ids_[i]],
                               path.path_points()[original_ids_[i + 1]]);
    }
    max_error_per_segment_.clear();
    max_error_per_segment_.reserve(num_points_ - 1);
    for (int i = 0; i < num_points_ - 1; ++i) {
        max_error_per_segment_.push_back(
            compute_max_error(path, original_ids_[i], original_ids_[i + 1]));
    }
}

bool PathApproximation::InitProjections(const Path &path) {
    if (num_points_ == 0) {
        return false;
    }
    projections_.clear();
    projections_.reserve(segments_.size() + 1);
    double s = 0.0;
    projections_.push_back(0);
    for (const auto &segment : segments_) {
        s += segment.length();
        projections_.push_back(s);
    }
    const auto &original_points = path.path_points();
    const int num_original_points = static_cast<int>(original_points.size());
    original_projections_.clear();
    original_projections_.reserve(num_original_points);
    for (size_t i = 0; i < projections_.size(); ++i) {
        original_projections_.push_back(projections_[i]);
        if (i + 1 < projections_.size()) {
            const auto &segment = segments_[i];
            for (int idx = original_ids_[i] + 1; idx < original_ids_[i + 1]; ++idx) {
                const double proj = segment.ProjectOntoUnit(original_points[idx]);
                original_projections_.push_back(
                    projections_[i] + std::max(0.0, std::min(proj, segment.length())));
            }
        }
    }

    // max_p_to_left[i] = max(p[0], p[1], ... p[i]).
    max_original_projections_to_left_.resize(num_original_points);
    double last_projection = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < num_original_points; ++i) {
        last_projection = std::max(last_projection, original_projections_[i]);
        max_original_projections_to_left_[i] = last_projection;
    }
    for (int i = 0; i + 1 < num_original_points; ++i) {
        if (!CHECK_LE(max_original_projections_to_left_[i],
                      max_original_projections_to_left_[i + 1] + kMathEpsilon)) {
            return false;
        }
    }

    // min_p_to_right[i] = min(p[i], p[i + 1], ... p[size - 1]).
    min_original_projections_to_right_.resize(original_projections_.size());
    last_projection = std::numeric_limits<double>::infinity();
    for (int i = num_original_points - 1; i >= 0; --i) {
        last_projection = std::min(last_projection, original_projections_[i]);
        min_original_projections_to_right_[i] = last_projection;
    }
    for (int i = 0; i + 1 < num_original_points; ++i) {
        if (!CHECK_LE(min_original_projections_to_right_[i],
                      min_original_projections_to_right_[i + 1] + kMathEpsilon)) {
            return false;
        }
    }

    // Sample max_p_to_left by sample_distance.
    max_projection_ = projections_.back();
    num_projection_samples_ =
        static_cast<int>(max_projection_ / kSampleDistance) + 1;
    sampled_max_original_projections_to_left_.clear();
    sampled_max_original_projections_to_left_.reserve(num_projection_samples_);
    double proj = 0.0;
    int last_index = 0;
    for (size_t i = 0; i < num_projection_samples_; ++i) {
        while (last_index + 1 < num_original_points &&
               max_original_projections_to_left_[last_index + 1] < proj) {
            ++last_index;
        }
        sampled_max_original_projections_to_left_.push_back(last_index);
        proj += kSampleDistance;
    }
    if (!CHECK_EQ(sampled_max_original_projections_to_left_.size(),
                  num_projection_samples_)) {
        return false;
    }

    return true;
}

bool PathApproximation::GetProjection(const Path &path, const Vec2d &point, double *accumulate_s, double *lateral, double *min_distance) const {
    if (num_points_ == 0) {
        return false;
    }
    if (accumulate_s == nullptr || lateral == nullptr ||
        min_distance == nullptr) {
        return false;
    }
    double min_distance_sqr = std::numeric_limits<double>::infinity();
    int estimate_nearest_segment_idx = -1;
    std::vector<double> distance_sqr_to_segments;
    distance_sqr_to_segments.reserve(segments_.size());
    for (size_t i = 0; i < segments_.size(); ++i) {
        const double distance_sqr = segments_[i].DistanceSquareTo(point);
        distance_sqr_to_segments.push_back(distance_sqr);
        if (distance_sqr < min_distance_sqr) {
            min_distance_sqr = distance_sqr;
            estimate_nearest_segment_idx = static_cast<int>(i);
        }
    }
    if (estimate_nearest_segment_idx < 0) {
        return false;
    }
    const auto &original_segments = path.segments();
    const int num_original_segments = static_cast<int>(original_segments.size());
    const auto &original_accumulated_s = path.accumulated_s();
    double min_distance_sqr_with_error =
        Sqr(sqrt(min_distance_sqr) +
            max_error_per_segment_[estimate_nearest_segment_idx] + max_error_);
    *min_distance = std::numeric_limits<double>::infinity();
    int nearest_segment_idx = -1;
    for (size_t i = 0; i < segments_.size(); ++i) {
        if (distance_sqr_to_segments[i] >= min_distance_sqr_with_error) {
            continue;
        }
        int first_segment_idx = original_ids_[i];
        int last_segment_idx = original_ids_[i + 1] - 1;
        double max_original_projection = std::numeric_limits<double>::infinity();
        if (first_segment_idx < last_segment_idx) {
            const auto &segment = segments_[i];
            const double projection = segment.ProjectOntoUnit(point);
            const double prod_sqr = Sqr(segment.ProductOntoUnit(point));
            if (prod_sqr >= min_distance_sqr_with_error) {
                continue;
            }
            const double scan_distance = sqrt(min_distance_sqr_with_error - prod_sqr);
            const double min_projection = projection - scan_distance;
            max_original_projection = projections_[i] + projection + scan_distance;
            if (min_projection > 0.0) {
                const double limit = projections_[i] + min_projection;
                const size_t sample_index =
                    std::max(0UL, static_cast<size_t>(limit / kSampleDistance));
                if (sample_index >= num_projection_samples_) {
                    first_segment_idx = last_segment_idx;
                } else {
                    first_segment_idx =
                        std::max(first_segment_idx,
                                 sampled_max_original_projections_to_left_[sample_index]);
                    if (first_segment_idx >= last_segment_idx) {
                        first_segment_idx = last_segment_idx;
                    } else {
                        while (first_segment_idx < last_segment_idx &&
                               max_original_projections_to_left_[first_segment_idx + 1] <
                                   limit) {
                            ++first_segment_idx;
                        }
                    }
                }
            }
        }
        bool min_distance_updated = false;
        bool is_within_end_point = false;
        for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
            if (min_original_projections_to_right_[idx] > max_original_projection) {
                break;
            }
            const auto &original_segment = original_segments[idx];
            const double x0 = point.x() - original_segment.start().x();
            const double y0 = point.y() - original_segment.start().y();
            const double ux = original_segment.unit_direction().x();
            const double uy = original_segment.unit_direction().y();
            double proj = x0 * ux + y0 * uy;
            double distance = 0.0;
            if (proj < 0.0) {
                if (is_within_end_point) {
                    continue;
                }
                is_within_end_point = true;
                distance = hypot(x0, y0);
            } else if (proj <= original_segment.length()) {
                is_within_end_point = true;
                distance = std::abs(x0 * uy - y0 * ux);
            } else {
                is_within_end_point = false;
                if (idx != last_segment_idx) {
                    continue;
                }
                distance = original_segment.end().DistanceTo(point);
            }
            if (distance < *min_distance) {
                min_distance_updated = true;
                *min_distance = distance;
                nearest_segment_idx = idx;
            }
        }
        if (min_distance_updated) {
            min_distance_sqr_with_error = Sqr(*min_distance + max_error_);
        }
    }
    if (nearest_segment_idx >= 0) {
        const auto &segment = original_segments[nearest_segment_idx];
        double proj = segment.ProjectOntoUnit(point);
        const double prod = segment.ProductOntoUnit(point);
        if (nearest_segment_idx > 0) {
            proj = std::max(0.0, proj);
        }
        if (nearest_segment_idx + 1 < num_original_segments) {
            proj = std::min(segment.length(), proj);
        }
        *accumulate_s = original_accumulated_s[nearest_segment_idx] + proj;
        if ((nearest_segment_idx == 0 && proj < 0.0) ||
            (nearest_segment_idx + 1 == num_original_segments &&
             proj > segment.length())) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0 ? (*min_distance) : -(*min_distance));
        }
        return true;
    }
    return false;
}

bool Path::GetKappaFromS(const double s, double *kappa) {
    if (kappa == nullptr) {
        AERROR << "Error: Null pointer provided for kappa.";
        return false;
    }

    int index = static_cast<int>(GetIndexFromS(s).id);

    if (index < 0) {
        *kappa = kappas_[0];
        AERROR << "Error: Index out of bounds. Index: " << index << ", Size: " << kappas_.size();
        return false;
    }
    if (index >= kappas_.size()) {
        *kappa = kappas_[kappas_.size() - 1];
        AERROR << "Error: Index out of bounds. Index: " << index << ", Size: " << kappas_.size();
        return false;
    }

    // 如果索引指向数组的最后一个元素，无法进行插值
    if (index == kappas_.size() - 1) {
        *kappa = kappas_[index];
        return true;
    }

    // 获取当前索引和下一个索引的曲率值
    double kappa1 = kappas_[index];
    double kappa2 = kappas_[index + 1];

    // 计算插值比例
    double s1 = accumulated_s_[index];
    double s2 = accumulated_s_[index + 1];
    if (s2 == s1) {
        AERROR << "Error: Division by zero in interpolation.";
        return false;
    }
    double t = (s - s1) / (s2 - s1);

    // 进行线性插值并返回结果
    *kappa = kappa1 + t * (kappa2 - kappa1);
    return true;
}
bool Path::computePathProfile(const std::vector<Vec2d> &path_points) {
    headings_.clear();
    kappas_.clear();
    dkappas_.clear();

    if (path_points.size() < 2) {
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
    std::size_t points_size = path_points.size();
    for (std::size_t i = 0; i < points_size; ++i) {
        double x_delta = 0.0;
        double y_delta = 0.0;
        if (i == 0) {
            x_delta = (path_points[i + 1].x() - path_points[i].x());
            y_delta = (path_points[i + 1].y() - path_points[i].y());
        } else if (i == points_size - 1) {
            x_delta = (path_points[i].x() - path_points[i - 1].x());
            y_delta = (path_points[i].y() - path_points[i - 1].y());
        } else {
            x_delta = 0.5 * (path_points[i + 1].x() - path_points[i - 1].x());
            y_delta = 0.5 * (path_points[i + 1].y() - path_points[i - 1].y());
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
    double fx = path_points[0].x();
    double fy = path_points[0].y();
    double nx = 0.0;
    double ny = 0.0;
    for (std::size_t i = 1; i < points_size; ++i) {
        nx = path_points[i].x();
        ny = path_points[i].y();
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
            xds = (path_points[i + 1].x() - path_points[i].x()) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
            yds = (path_points[i + 1].y() - path_points[i].y()) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i));
        } else if (i == points_size - 1) {
            xds = (path_points[i].x() - path_points[i - 1].x()) /
                  (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
            yds = (path_points[i].y() - path_points[i - 1].y()) /
                  (accumulated_s_.at(i) - accumulated_s_.at(i - 1));
        } else {
            xds = (path_points[i + 1].x() - path_points[i - 1].x()) /
                  (accumulated_s_.at(i + 1) - accumulated_s_.at(i - 1));
            yds = (path_points[i + 1].y() - path_points[i - 1].y()) /
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
