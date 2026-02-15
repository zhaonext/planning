#pragma once

#include <string>
#include <utility>
#include <vector>

#include "Common/common.h"
#include "aaboxkdtree2d.h"
#include "box2d.h"
#include "line_segment2d.h"
#include "vec2d.h"

template <class Object, class GeoObject>
class ObjectWithAABox {
public:
    ObjectWithAABox(const AABox2d &aabox, const Object *object,
                    const GeoObject *geo_object, const int id)
        : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
    ~ObjectWithAABox() {}
    const AABox2d &aabox() const { return aabox_; }
    double DistanceTo(const Vec2d &point) const {
        return geo_object_->DistanceTo(point);
    }
    double DistanceSquareTo(const Vec2d &point) const {
        return geo_object_->DistanceSquareTo(point);
    }
    const Object *object() const { return object_; }
    const GeoObject *geo_object() const { return geo_object_; }
    int id() const { return id_; }

private:
    AABox2d aabox_;
    const Object *object_;
    const GeoObject *geo_object_;
    int id_;
};

using PathSegmentBox = ObjectWithAABox<void, LineSegment2d>;
using PathSegmentKDTree = AABoxKDTree2d<PathSegmentBox>;

class Path;

class PathApproximation {
public:
    PathApproximation() = default;
    PathApproximation(const Path &path, const double max_error)
        : max_error_(max_error), max_sqr_error_(max_error * max_error) {
        Init(path);
    }
    double max_error() const { return max_error_; }
    const std::vector<int> &original_ids() const { return original_ids_; }
    const std::vector<LineSegment2d> &segments() const {
        return segments_;
    }

    bool GetProjection(const Path &path, const Vec2d &point,
                       double *accumulate_s, double *lateral,
                       double *distance) const;

    bool OverlapWith(const Path &path, const Box2d &box,
                     double width) const;

protected:
    void Init(const Path &path);
    bool is_within_max_error(const Path &path, const int s, const int t);
    double compute_max_error(const Path &path, const int s, const int t);

    void InitDilute(const Path &path);
    bool InitProjections(const Path &path);

protected:
    double max_error_ = 0;
    double max_sqr_error_ = 0;

    int num_points_ = 0;
    std::vector<int> original_ids_;
    std::vector<LineSegment2d> segments_;
    std::vector<double> max_error_per_segment_;

    // TODO(All): use direction change checks to early stop.

    // Projection of points onto the diluated segments.
    std::vector<double> projections_;
    double max_projection_;
    size_t num_projection_samples_ = 0;

    // The original_projection is the projection of original points onto the
    // diluated segments.
    std::vector<double> original_projections_;
    // max_p_to_left[i] = max(p[0], p[1], ... p[i]).
    // min_p_to_right[i] = min(p[i], p[i + 1], ... p[size - 1]).
    std::vector<double> max_original_projections_to_left_;
    std::vector<double> min_original_projections_to_right_;
    std::vector<int> sampled_max_original_projections_to_left_;
};

class InterpolatedIndex {
public:
    InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
    int id = 0;
    double offset = 0.0;
};

class Path {
public:
    explicit Path(const std::vector<Vec2d> &path_points) : path_points_(path_points) {
        Init();
        CreateKDTree();
        computePathProfile(path_points);
    };

    // Compute accumulate s value of the index.
    double GetSFromIndex(const InterpolatedIndex &index) const;
    // Compute interpolated index by accumulate_s.
    InterpolatedIndex GetIndexFromS(double s) const;

    int GetNearestPoint(const Vec2d &point) const;

    bool GetNearestPoint(const Vec2d &point, double *accumulate_s,
                         double *lateral) const;
    bool GetNearestPoint(const Vec2d &point, double *accumulate_s,
                         double *lateral, double *distance) const;
    bool GetProjectionWithHueristicParams(const Vec2d &point,
                                          const double hueristic_start_s,
                                          const double hueristic_end_s,
                                          double *accumulate_s, double *lateral,
                                          double *min_distance) const;
    bool GetProjection(const Vec2d &point, double *accumulate_s,
                       double *lateral) const;
    bool GetProjection(const Vec2d &point, double *accumulate_s,
                       double *lateral, double *distance) const;
    bool GetProjectionQuick(const Vec2d &point, double *accumulate_s,
                            double *lateral, double *min_distance, bool first_search = false) const;

    bool GetHeadingAlongPath(const Vec2d &point, double *heading) const;

    int num_points() const { return num_points_; }
    int num_segments() const { return num_segments_; }
    const std::vector<Vec2d> &path_points() const { return path_points_; }

    const std::vector<Vec2d> &unit_directions() const {
        return unit_directions_;
    }
    const std::vector<double> &accumulated_s() const { return accumulated_s_; }
    const std::vector<LineSegment2d> &segments() const {
        return segments_;
    }
    const PathApproximation *approximation() const { return &approximation_; }
    double length() const { return length_; }
    void CreateKDTree();

    State calInitState(const State &X) {
        Eigen::Matrix<double, NUM_STATE, 1> result;

        // 计算 t 值
        double x_ego = X.x, y_ego = X.y, v_ego = X.v, phi_ego = X.phi;
        //        double t = minDisToBezierIndexT(x_ego, y_ego);
        // 计算 s 值
        double s = 0.0;
        double n = 0.0;
        //        double heading = 0.0;
        //        GetHeadingAlongPath({x_ego, y_ego}, &heading);
        GetProjection({x_ego, y_ego}, &s, &n);
        //        double s = calArcLength(t);

        result(0) = s;

        // 计算 en 向量
        //        std::vector<double> d = evaluateFirstDerivative(t);
        Vec2d tangent_direction;
        GetHeadingAlongPath({x_ego, y_ego}, &tangent_direction);
        Eigen::Vector2d tangentDirection(tangent_direction.x(), tangent_direction.y());
        tangentDirection.normalize();
        Eigen::Matrix2d rotation90;
        rotation90 << 0, -1, 1, 0;
        Eigen::Vector2d en = rotation90 * tangentDirection;
        en.normalize();

        // 计算 n 值
        //        auto ref_point = evaluate(t);
        //        double r_x = ref_point.at(0);
        //        double r_y = ref_point.at(1);
        //        Eigen::Vector2d err{x_ego - r_x, y_ego - r_y};
        //        double n = err.transpose() * en;
        result(1) = n;

        // 计算 alpha 值
        //        double ref_phi = calTangentAngle(t);
        double heading;
        GetHeadingAlongPath({x_ego, y_ego}, &heading);
        //    double alpha = phi_ego - M_PI + ref_phi;
        //    double alpha = phi_ego + std::abs(M_PI + ref_phi);
        double alpha = phi_ego - heading;
        result(2) = alpha;

        // 自车在笛卡尔坐标系下的状态
        result(3) = x_ego;
        result(4) = y_ego;
        result(5) = v_ego;
        result(6) = phi_ego;

        auto state = vectorToState(result);
        return state;
    }

    bool GetKappaFromS(double s, double *kappa);
    bool computePathProfile(const std::vector<Vec2d> &path_points);
    bool GetHeadingAlongPath(const Vec2d &point, Vec2d *tangentDirection) const;
    Vec2d GetEndPoint() { return path_points_.at(path_points_.size() - 1); };

protected:
    bool Init();
    bool InitPoints();
    bool InitPointIndex();

protected:
    size_t num_points_ = 0;
    size_t num_segments_ = 0;
    std::vector<Vec2d> path_points_;

    std::vector<double> lane_accumulated_s_;
    std::vector<Vec2d> unit_directions_;
    double length_ = 0.0;
    std::vector<double> accumulated_s_;
    std::vector<LineSegment2d> segments_;
    bool use_path_approximation_ = false;
    PathApproximation approximation_;

    // Sampled every fixed length.
    size_t num_sample_points_ = 0;
    std::vector<double> lane_left_width_;
    std::vector<double> lane_right_width_;
    std::vector<double> road_left_width_;
    std::vector<double> road_right_width_;
    std::vector<int> last_point_index_;
    // new HYK
    bool UsePerMap_ = false;

    std::vector<PathSegmentBox> segment_box_list_;
    std::shared_ptr<PathSegmentKDTree> path_segment_kdtree_;

    std::vector<double> headings_;
    //    std::vector<double> accumulated_s_;
    std::vector<double> kappas_;
    std::vector<double> dkappas_;
};
