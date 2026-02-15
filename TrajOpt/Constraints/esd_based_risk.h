#pragma once

#include <gsl/gsl_spline2d.h>

#include <cmath>

#include "temp_file/vehicle_dynamic.h"

namespace planning {

constexpr double x_min_ = -10;
constexpr double x_max_ = 20;
constexpr double y_min_ = -10;
constexpr double y_max_ = 10;
constexpr double x_res_ = 0.05;
constexpr double y_res_ = 0.05;
constexpr int x_size_ = (x_max_ - x_min_) / x_res_ + 1;
constexpr int y_size_ = (y_max_ - y_min_) / y_res_ + 1;

class SDF {
    double x_data_[x_size_]{};
    double y_data_[y_size_]{};
    double z_data_[x_size_ * y_size_]{};
    gsl_spline2d *spline_{};
    gsl_interp_accel *xacc_{};
    gsl_interp_accel *yacc_{};

public:
    SDF() {
        Eigen::Map<Eigen::Matrix<double, x_size_, 1>> xdata_eigen(x_data_);
        xdata_eigen = Eigen::VectorXd::LinSpaced(x_size_, x_min_, x_max_);

        Eigen::Map<Eigen::Matrix<double, y_size_, 1>> ydata_eigen(y_data_);
        ydata_eigen = Eigen::VectorXd::LinSpaced(y_size_, y_min_, y_max_);

        for (int i = 0; i < x_size_; i++) {
            for (int j = 0; j < y_size_; j++) {
                double dx, dy, ddx, ddy, dxdy;
                double x = x_data_[i], y = y_data_[j];
                auto dist = GetFieldAndGradient(x, y, dx, dy, ddx, ddy, dxdy);
                auto k = j * x_size_ + i;
                z_data_[k] = dist;
            }
        }

        spline_ = gsl_spline2d_alloc(gsl_interp2d_bicubic, x_size_, y_size_);
        gsl_spline2d_init(spline_, x_data_, y_data_, z_data_, x_size_, y_size_);
        xacc_ = gsl_interp_accel_alloc();
        yacc_ = gsl_interp_accel_alloc();
    }

    ~SDF() {
        gsl_spline2d_free(spline_);
        gsl_interp_accel_free(xacc_);
        gsl_interp_accel_free(yacc_);
    }

    static double GetFieldAndGradient(double x, double y, double &dx, double &dy, double &ddx, double &ddy, double &dxdy) {
        constexpr double cx = 2;
        constexpr double cy = 2;
        constexpr double radius = 3;
        auto dist_x = x - cx;
        auto dist_y = y - cy;
        auto d = hypot(dist_x, dist_y);
        dx = dist_x / d;
        dy = dist_y / d;
        auto d3 = d * d * d;
        ddx = -dist_x * dist_x / d3 + 1 / d;
        ddy = -dist_y * dist_y / d3 + 1 / d;
        dxdy = -dist_x * dist_y / d3;
        return d - radius;
    }

    double GetFieldAndGradient1(double x, double y, double &dx, double &dy, double &ddx, double &ddy, double &dxdy) {
        double dist;
        dist = gsl_spline2d_eval(spline_, x, y, xacc_, yacc_);
        dx = gsl_spline2d_eval_deriv_x(spline_, x, y, xacc_, yacc_);
        dy = gsl_spline2d_eval_deriv_y(spline_, x, y, xacc_, yacc_);
        ddx = gsl_spline2d_eval_deriv_xx(spline_, x, y, xacc_, yacc_);
        ddy = gsl_spline2d_eval_deriv_yy(spline_, x, y, xacc_, yacc_);
        dxdy = gsl_spline2d_eval_deriv_xy(spline_, x, y, xacc_, yacc_);
        return dist;
    }
};

class VehicleBodyFrame {
    double x_body_{};
    double y_body_{};
    double x_{};
    double y_{};
    double gradient_x_[3] = {1, 0, 0};
    double gradient_y_[3] = {0, 1, 0};
    Eigen::Matrix3d hessian_x_;
    Eigen::Matrix3d hessian_y_;

public:
    VehicleBodyFrame() = default;

    VehicleBodyFrame(double x, double y) : x_body_(x), y_body_(y) {}

    void SetPoint(double x_body, double y_body) {
        x_body_ = x_body;
        y_body_ = y_body;
    }

    void UpdateVehiclePosition(double x, double y, double cos_theta, double sin_theta) {
        x_ = x + cos_theta * x_body_ - sin_theta * y_body_;
        y_ = y + sin_theta * x_body_ + cos_theta * y_body_;

        gradient_x_[2] = -sin_theta * x_body_ - cos_theta * y_body_;
        gradient_y_[2] = cos_theta * x_body_ - sin_theta * y_body_;

        hessian_x_.setZero();
        hessian_x_(2, 2) = -cos_theta * x_body_ + sin_theta * y_body_;

        hessian_y_.setZero();
        hessian_y_(2, 2) = -sin_theta * x_body_ - cos_theta * y_body_;
    }

    double GetX() const { return x_; }
    double GetY() const { return y_; }
    const double *GetGradientX() const { return gradient_x_; }
    const double *GetGradientY() const { return gradient_y_; }
    const double *GetHessianX() const { return hessian_x_.data(); }
    const double *GetHessianY() const { return hessian_y_.data(); }
};

template <int N = 2>
class SDFRiskEvaluator {
    VehicleBodyFrame disk_centers_[N]{};
    double risk_{};
    double risk_gradient_[3]{};
    double risk_hessian_[3][3]{};
    static constexpr double mu_ = 1;
    std::shared_ptr<SDF> sdf_ptr_;

public:
    explicit SDFRiskEvaluator() {
        double radius = hypot(VEHICLE_L / N / 2, VEHICLE_W / 2);
        double center_x = -VEHICLE_LB + VEHICLE_L / N / 2;
        for (auto &disk : disk_centers_) {
            disk.SetPoint(center_x, 0);
            center_x += radius;
        }
        sdf_ptr_ = std::make_shared<SDF>();
    }

    void UpdateVehiclePosition(double x, double y, double cos_theta, double sin_theta) {
        // reset risk value and its gradient
        risk_ = 0;
        for (auto &g : risk_gradient_) {
            g = 0;
        }
        Eigen::Map<Eigen::Matrix3d> H(&risk_hessian_[0][0]);
        H.setZero();

        double risk_at_disk[N]{};
        double sdf_at_disk[N]{};
        double sdf_gradient_at_disk[N][3]{};
        double sdf_hessian_at_disk[N][3][3]{};
        for (int disk_index = 0; disk_index < N; disk_index++) {
            // update disk position
            auto &disk = disk_centers_[disk_index];
            disk.UpdateVehiclePosition(x, y, cos_theta, sin_theta);
            double dx, dy, ddx, ddy, dxdy;
            //                 auto sdf = SDF::GetFieldAndGradient(disk.GetX(), disk.GetY(), dx, dy, ddx, ddy, dxdy);
            auto sdf = sdf_ptr_->GetFieldAndGradient1(disk.GetX(), disk.GetY(), dx, dy, ddx, ddy, dxdy);

            // sdf
            sdf_at_disk[disk_index] = sdf;

            // sdf gradient to vehicle position
            for (int i = 0; i < 3; i++) {
                sdf_gradient_at_disk[disk_index][i] = dx * disk.GetGradientX()[i] + dy * disk.GetGradientY()[i];
            }

            // sdf hessian to vehicle position
            Eigen::Map<Eigen::Vector3d> G_disk_x((double *)disk.GetGradientX());
            Eigen::Map<Eigen::Vector3d> G_disk_y((double *)disk.GetGradientY());
            Eigen::Map<Eigen::Matrix3d> H_disk_x((double *)disk.GetHessianX());
            Eigen::Map<Eigen::Matrix3d> H_disk_y((double *)disk.GetHessianY());
            for (int i = 0; i < 3; i++) {
                for (int j = i; j < 3; j++) {
                    sdf_hessian_at_disk[disk_index][i][j] = G_disk_x[i] * (ddx * G_disk_x[j] + dxdy * G_disk_y[j]) +
                                                            G_disk_y[i] * (ddy * G_disk_y[j] + dxdy * G_disk_x[j]) +
                                                            dx * H_disk_x(i, j) + dy * H_disk_y(i, j);
                }
            }
            // flip at diag
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < i; j++) {
                    sdf_hessian_at_disk[disk_index][i][j] = sdf_hessian_at_disk[disk_index][j][i];
                }
            }
        }

        for (int disk_index = 0; disk_index < N; disk_index++) {
            // effect on risk should sum up
            risk_at_disk[disk_index] = exp(-mu_ * sdf_at_disk[disk_index]);
            risk_ += risk_at_disk[disk_index];
        }

        for (int i = 0; i < 3; i++) {
            for (int disk_index = 0; disk_index < N; disk_index++) {
                // effect on gradient should sum up
                risk_gradient_[i] += (-mu_ * risk_at_disk[disk_index] * sdf_gradient_at_disk[disk_index][i]);
            }
        }

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int disk_index = 0; disk_index < N; disk_index++) {
                    // effect on hessian should sum up
                    risk_hessian_[i][j] += (-mu_ * risk_at_disk[disk_index] * sdf_hessian_at_disk[disk_index][i][j] +
                                            mu_ * mu_ * risk_at_disk[disk_index] * sdf_gradient_at_disk[disk_index][i] * sdf_gradient_at_disk[disk_index][j]);
                }
            }
        }
    }

    double GetRisk() const { return risk_; }
    const double *GetRiskGradient() const { return risk_gradient_; }
    const double *GetRiskHessian() const { return &risk_hessian_[0][0]; }
};
}  // namespace planning
