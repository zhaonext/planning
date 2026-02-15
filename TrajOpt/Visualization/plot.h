#pragma once

#include "Common/types.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

static std::string formatDouble(double value, int precision = 2) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

class Plot {
public:
    static void drawZebraCrossing(double start_x, double start_y, double length, double width, double stripe_width, double stripe_spacing, double yaw) {
        // 遍历斑马线的长度，绘制每个条纹
        for (double pos = 0; pos < length; pos += stripe_spacing + stripe_width) {
            // 计算条纹的起始和结束位置
            double stripe_start_x = start_x + pos * cos(yaw);
            double stripe_start_y = start_y + pos * sin(yaw);
            double stripe_end_x = start_x + (pos + stripe_width) * cos(yaw);
            double stripe_end_y = start_y + (pos + stripe_width) * sin(yaw);

            // 计算条纹的四个角的坐标
            std::vector<double> x_points = {
                stripe_start_x - width / 2 * sin(yaw),
                stripe_end_x - width / 2 * sin(yaw),
                stripe_end_x + width / 2 * sin(yaw),
                stripe_start_x + width / 2 * sin(yaw)
            };

            std::vector<double> y_points = {
                stripe_start_y + width / 2 * cos(yaw),
                stripe_end_y + width / 2 * cos(yaw),
                stripe_end_y - width / 2 * cos(yaw),
                stripe_start_y - width / 2 * cos(yaw)
            };

            // 使用 fill 函数填充条纹
            std::map<std::string, std::string> keywords;
            keywords["color"] = "k";
            plt::fill(x_points, y_points, keywords);  // 使用黑色填充
        }
    }

    static void draw_road(std::vector<double> &x_center, std::vector<double> &y_center,
                          std::vector<double> &x_left, std::vector<double> &y_left,
                          std::vector<double> &x_right, std::vector<double> &y_right,
                          const std::string &color) {
        if (x_center.size() % 2 != 0) {
            x_center.pop_back();
            y_center.pop_back();
        }
        if (x_left.size() % 2 != 0) {
            x_left.pop_back();
            y_left.pop_back();
        }
        if (x_right.size() % 2 != 0) {
            x_right.pop_back();
            y_right.pop_back();
        }

        std::map<std::string, std::string> ref_keywords;
        ref_keywords["color"] = color;
        ref_keywords["ls"] = "--";
        ref_keywords["label"] = "ref_line";
        plt::plot(x_center, y_center, ref_keywords);

        std::map<std::string, std::string> bound_keywords;
        bound_keywords["color"] = color;
        bound_keywords["ls"] = "-";
        plt::plot(x_left, y_left, bound_keywords);
        plt::plot(x_right, y_right, bound_keywords);

        plt::xlabel("X(m)");
        plt::ylabel("Y(m)");
        plt::legend();
    }
    static void draw_optimize_trajectory(const Trajectory &trajectory, const std::string &color) {
        auto traj_point = trajectory.points;
        std::vector<double> opt_x, opt_y;
        for (const auto &point : traj_point) {
            opt_x.emplace_back(point.x);
            opt_y.emplace_back(point.y);
        }
        std::map<std::string, std::string> opt_keywords;
        opt_keywords["color"] = color;
        opt_keywords["ls"] = "-";
        opt_keywords["label"] = "optimize_trajectory";
        plt::plot(opt_x, opt_y, opt_keywords);
    }
    static void draw_ego_rectangle(double rear_x, double rear_y, double theta, const std::string &color) {
        double car_length_ = WHEELBASE + REAR_SUSPENSION + FRONT_SUSPENSION;
        double car_width_ = VEHICLE_WIDTH;
        double x_center = rear_x + WHEELBASE / 2 * cos(theta);
        double y_center = rear_y + WHEELBASE / 2 * sin(theta);
        std::array<double, 5> car_x_{};
        std::array<double, 5> car_y_{};

        car_x_ = {x_center - car_length_ / 2, x_center + car_length_ / 2, x_center + car_length_ / 2,
                  x_center - car_length_ / 2, x_center - car_length_ / 2};
        car_y_ = {y_center - car_width_ / 2, y_center - car_width_ / 2, y_center + car_width_ / 2,
                  y_center + car_width_ / 2, y_center - car_width_ / 2};

        for (int i = 0; i < 5; ++i) {
            double temp_x = car_x_[i];
            car_x_[i] = (temp_x - x_center) * cos(theta) - (car_y_[i] - y_center) * sin(theta) + x_center;
            car_y_[i] = (temp_x - x_center) * sin(theta) + (car_y_[i] - y_center) * cos(theta) + y_center;
        }
        plt::plot({car_x_[0], car_x_[1], car_x_[2], car_x_[3], car_x_[4]},
                  {car_y_[0], car_y_[1], car_y_[2], car_y_[3], car_y_[4]}, color + "-");
        plt::axis("equal");
    }
    static void draw_obs_rectangle(const TrajectoryPoint &vehicle_state, const int obs_id, ObstacleType obs_type, ObstacleIntent obs_intent,
                                   const double obs_probability, const std::pair<double, double> &ellipse_param) {
        double rear_x = vehicle_state.x;
        double rear_y = vehicle_state.y;
        double theta = vehicle_state.phi;
        auto ellipse_a = ellipse_param.first;
        auto ellipse_b = ellipse_param.second;
        double car_length_ = WHEELBASE + REAR_SUSPENSION + FRONT_SUSPENSION;
        double car_width_ = VEHICLE_WIDTH;
        double x_center = rear_x + WHEELBASE / 2 * cos(theta);
        double y_center = rear_y + WHEELBASE / 2 * sin(theta);
        std::array<double, 5> car_x_{};
        std::array<double, 5> car_y_{};

        car_x_ = {x_center - car_length_ / 2, x_center + car_length_ / 2, x_center + car_length_ / 2,
                  x_center - car_length_ / 2, x_center - car_length_ / 2};
        car_y_ = {y_center - car_width_ / 2, y_center - car_width_ / 2, y_center + car_width_ / 2,
                  y_center + car_width_ / 2, y_center - car_width_ / 2};

        for (int i = 0; i < 5; ++i) {
            double temp_x = car_x_[i];
            car_x_[i] = (temp_x - x_center) * cos(theta) - (car_y_[i] - y_center) * sin(theta) + x_center;
            car_y_[i] = (temp_x - x_center) * sin(theta) + (car_y_[i] - y_center) * cos(theta) + y_center;
        }

        std::string color = getColorStringByObstacleType(obs_type);
        std::string intent = getObstacleIntentString(obs_intent);
        std::string probability = formatDouble(obs_probability);

        plt::plot({car_x_[0], car_x_[1], car_x_[2], car_x_[3], car_x_[4]},
                  {car_y_[0], car_y_[1], car_y_[2], car_y_[3], car_y_[4]}, color + "-");
        plt::axis("equal");
        plt::text(x_center, y_center, std::to_string(obs_id) + ": " + intent + ": " + probability);
        draw_ellipse(rear_x, rear_y, theta, ellipse_a, ellipse_b, color);
    }
    static void draw_ellipse(double rear_x, double rear_y, double theta, double a, double b, const std::string &color) {
        std::vector<double> x_vals, y_vals;
        double x_center = rear_x + WHEELBASE / 2 * cos(theta);
        double y_center = rear_y + WHEELBASE / 2 * sin(theta);
        // 生成椭圆上的点
        for (double t = 0; t <= 2 * M_PI; t += 0.01) {
            double x_ellipse = a * cos(t);
            double y_ellipse = b * sin(t);

            // 旋转并平移椭圆
            double x_rotated = x_ellipse * cos(theta) - y_ellipse * sin(theta) + x_center;
            double y_rotated = x_ellipse * sin(theta) + y_ellipse * cos(theta) + y_center;

            x_vals.push_back(x_rotated);
            y_vals.push_back(y_rotated);
        }
        plt::plot(x_vals, y_vals, color + "-");
        plt::axis("equal");
    }

    static void draw_velocity_curve(const std::vector<double> &t_s, const std::vector<double> &velocity) {
        plt::named_plot("velocity", t_s, velocity, "b-");

        std::map<std::string, std::string> keywords;
        keywords["color"] = "r";
        keywords["ls"] = "--";
        plt::axhline(DESIRED_SPEED * 3.6, keywords);

        plt::grid(true);
        plt::ylabel("V (km/h)");
        plt::ylim(0.0, DESIRED_SPEED * 3.6 + 10);
        plt::legend();
    }
    static void draw_delta_curve(const std::vector<double> &t_s, const std::vector<double> &delta) {
        plt::named_plot("delta", t_s, delta, "b-");

        std::map<std::string, std::string> keywords;
        keywords["color"] = "r";
        keywords["ls"] = "--";
        plt::axhline(MINIMUM_FRONT_WHEEL_TURNING_ANGLE * 180 / M_PI, keywords);
        plt::axhline(MAXIMUM_FRONT_WHEEL_TURNING_ANGLE * 180 / M_PI, keywords);

        plt::grid(true);
        plt::ylabel("Delta (°)");
        plt::ylim((MINIMUM_FRONT_WHEEL_TURNING_ANGLE - 0.1) * 180 / M_PI, (MAXIMUM_FRONT_WHEEL_TURNING_ANGLE + 0.1) * 180 / M_PI);
        plt::legend();
    }
    static void draw_acc_curve(const std::vector<double> &t_s, const std::vector<double> &acc) {
        plt::named_plot("acc", t_s, acc, "b-");

        std::map<std::string, std::string> keywords;
        keywords["color"] = "r";
        keywords["ls"] = "--";
        plt::axhline(MINIMUM_ACCELERATION, keywords);
        plt::axhline(MAXIMUM_ACCELERATION, keywords);

        plt::grid(true);
        plt::xlabel("Time(s)");
        plt::ylabel("Acc (m/s^2)");
        plt::legend();
    }
    static void draw_kappa_curve(const std::vector<double> &t_s, const std::vector<double> &kappa) {
        plt::named_plot("kappa", t_s, kappa, "b-");

        std::map<std::string, std::string> keywords;
        keywords["color"] = "r";
        keywords["ls"] = "--";
        plt::axhline(0.3, keywords);
        plt::axhline(-0.3, keywords);

        plt::grid(true);
        plt::xlabel("Time(s)");
        plt::ylabel("Kappa");
        plt::legend();
    }

    static std::string getColorStringByObstacleType(ObstacleType type) {
        switch (type) {
            case PEDESTRIAN:
                return "g";  // 绿色
            case BICYCLE:
                return "b";  // 蓝色
            case STATIC_VEHICLE:
                return "y";  // 黄色
            case DYNAMIC_VEHICLE:
                return "r";  // 红色
            case TYPE_UNKNOWN:
            default:
                return "gray";  // 灰色
        }
    }
    static std::string getObstacleIntentString(ObstacleIntent intent) {
        switch (intent) {
            case LF:
                return "LF";
            case LL:
                return "LL";
            case LR:
                return "LR";
            case STOP:
                return "Stop";
            case ACC:
                return "Acc";
            case DEC:
                return "Dec";
            case PARKING:
                return "Parking";
            case INTENT_UNKNOWN:
            default:
                return "Undefined";
        }
    }
};