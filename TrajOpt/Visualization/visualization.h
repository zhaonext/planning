#pragma once
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Common/common.h"

// 定义颜色结构体
constexpr struct Color {
    float r, g, b, a;
} RED = {1.0, 0.0, 0.0, 1.0},              // 纯红色
    GREEN = {0.0, 1.0, 0.0, 1.0},          // 纯绿色
    BLUE = {0.0, 0.0, 1.0, 1.0},           // 纯蓝色
    YELLOW = {1.0, 1.0, 0.0, 1.0},         // 明亮的黄色
    ORANGE = {1.0, 0.5, 0.0, 1.0},         // 明亮的橙色
    PURPLE = {0.5, 0.0, 0.5, 1.0},         // 深紫色
    CYAN = {0.0, 1.0, 1.0, 1.0},           // 青色，蓝绿色混合
    BLACK = {0.0, 0.0, 0.0, 1.0},          // 纯黑色
    WHITE = {1.0, 1.0, 1.0, 1.0},          // 纯白色
    GRAY = {0.5, 0.5, 0.5, 1.0},           // 中等灰色
    LIGHT_GRAY = {0.75, 0.75, 0.75, 1.0},  // 浅灰色
    DARK_GRAY = {0.25, 0.25, 0.25, 1.0},   // 深灰色
    PINK = {1.0, 0.0, 1.0, 1.0},           // 明亮的粉红色
    GOLD = {1.0, 0.84, 0.0, 1.0},          // 金色
    LIGHT_BLUE = {0.5, 0.5, 1.0, 1.0},     // 淡蓝色
    DARK_GREEN = {0.0, 0.4, 0.0, 1.0},     // 深绿色
    MAGENTA = {1.0, 0.0, 0.5, 1.0},        // 明亮的品红色
    TEAL = {0.0, 0.5, 0.5, 1.0},           // 青绿色
    OLIVE = {0.5, 0.5, 0.0, 1.0},          // 橄榄绿
    CORAL = {1.0, 0.5, 0.31, 1.0},         // 珊瑚色，一种淡橙色
    INDIGO = {0.29, 0.0, 0.51, 1.0},       // 靛蓝色，深蓝紫色
    LIME = {0.75, 1.0, 0.0, 1.0},          // 酸橙色，亮绿色
    BEIGE = {0.96, 0.96, 0.86, 1.0},       // 米色，淡黄色
    MAROON = {0.5, 0.0, 0.0, 1.0},         // 栗色，深红色
    TURQUOISE = {0.25, 0.88, 0.82, 1.0},   // 绿松石色，蓝绿色
    VIOLET = {0.56, 0.0, 1.0, 1.0};        // 紫罗兰色，深紫色

static Color getColorByObstacleType(ObstacleType type) {
    switch (type) {
        case PEDESTRIAN:
            return GREEN;  // 行人为绿色
        case BICYCLE:
            return PURPLE;  // 自行车为紫色
        case STATIC_VEHICLE:
            return YELLOW;  // 静止车辆为黄色
        case DYNAMIC_VEHICLE:
            return RED;  // 动态车辆为红色
        case TYPE_UNKNOWN:
        default:
            return GRAY;  // 未知类型或其他为灰色
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
static std::string changeDoubleFormat(double value, int precision = 2) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

class Visualization {
public:
    Visualization() : nh_("~"), is_map_load_(false) { initPublishers(); }
    void publishTrafficLightMarker(int id, double x, double y, double z, TrafficLightState state, TrafficLightOrientation orientation);

    /**
     * @brief zebra crossing publish: {marker}
     * @param crossing_id
     * @param start_x
     * @param start_y
     * @param length
     * @param width
     * @param stripe_width
     * @param stripe_spacing
     * @param yaw
     */
    void publishZebraCrossingMarker(int crossing_id, double start_x, double start_y, double length, double width, double stripe_width, double stripe_spacing, double yaw);

    /**
     * @brief road publish: {marker}
     * @param x_center
     * @param y_center
     * @param x_left
     * @param y_left
     * @param x_right
     * @param y_right
     * @param topic_name
     * @param color
     */
    void publishRoadMarkerWithScale(std::vector<double> &x_center, std::vector<double> &y_center,
                                    std::vector<double> &x_left, std::vector<double> &y_left,
                                    std::vector<double> &x_right, std::vector<double> &y_right,
                                    const std::string &topic_name, const Color &color) const;

    /**
     * @brief ego publish: {marker + trajectory + coordinate_system}
     * @param state
     * @param topic_name
     * @param color
     */
    void publishEgoStateMarker(const TrajectoryPoint &state, const std::string &topic_name, const Color &color);

    /**
     * @brief ego publish: {trajectory: 2D + 3D}
     * @param trajectory
     * @param topic_name
     * @param color
     */
    void publishEgoTrajectory(const std::vector<TrajectoryPoint> &trajectory, const std::string &topic_name, const Color &color) const;

    /**
     * @brief obstacle publish: {marker + Direction + {Intent + velocity + Probability text} + envelope_polygon}
     * @param state
     * @param id
     * @param obs_type
     * @param obs_intent
     * @param obs_probability
     * @param topic_name
     * @param ellipse_param
     * @param transparency
     */
    void publishTargetObstacleMarker(const TrajectoryPoint &state, int id, ObstacleType obs_type, ObstacleIntent obs_intent,
                                     double obs_probability, const std::string &topic_name, const std::pair<double, double> &ellipse_param, float transparency);
    void publishNoTargetObstacleMarker(const TrajectoryPoint &state, int id, ObstacleType obs_type, ObstacleIntent obs_intent,
                                       double obs_probability, const std::string &topic_name, const std::pair<double, double> &ellipse_param, float transparency);
    /**
     * @brief dashboard:
     *          1.speed + steering_angle
     *          2.time + Gear
     */
    void publishEgoSpeedAndSteeringAngleAndAcceleration(double speed, double steering_angle, double acceleration);
    void publishTimeAndGearOverlay(const std::string &time, const std::string &gear);

    /**
     * @brief global coordinate system
     */
    void publishStaticAxes();

    /**
     * @brief
     * @param trajectory
     * @param id
     * @param topic_name
     * @param color
     */
    void publishAllObstacleTrajectory(const vector<TrajectoryPoint> &trajectory, int id, const string &topic_name, const Color &color, float transparency = 1.0) const;

    void publishStopLineMarker(bool reach_stop_line, double x, double y, double yaw, double length, double width, double height, const Color &color);

    void publishSpeedProfile(const std::vector<std::tuple<double, double, double>> &speed_profile, const std::string &name, int id, const Color &color);
    void publishEgoReferenceLine(const std::vector<Vec2d> &line, const std::string &topic_name, const Color &color);

private:
    ros::NodeHandle nh_;
    bool is_map_load_;

    // Publishers
    ros::Publisher route_pub_;
    ros::Publisher waypoints_pub_;
    ros::Publisher ego_state_pub_;
    ros::Publisher target_obs_state_pub_;
    ros::Publisher no_target_obs_state_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher speed_pub_;
    ros::Publisher steering_angle_pub_;
    ros::Publisher acceleration_pub_;
    ros::Publisher time_gear_pub_;
    ros::Publisher global_axes_pub_;
    ros::Publisher ego_axes_pub_;
    ros::Publisher road_marker_array_pub_;
    ros::Publisher zebra_crossing_pub_;
    ros::Publisher traffic_light_pub_;
    ros::Publisher stop_line_pub_;
    ros::Publisher speed_profile_pub_;
    // tf2
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Markers
    visualization_msgs::Marker route_marker_;
    visualization_msgs::Marker waypoints_marker_;
    std::vector<geometry_msgs::Point> ego_3d_trajectory_points_;
    std::unordered_map<int, std::vector<geometry_msgs::Point>> obs_trajectories_;

    void initPublishers();

    // marker settings
    static void setDashedLineMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double scale, const Color &color, float transparency = 1.0);
    static void setSolidLineMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double scale, const Color &color, float transparency = 1.0);
    static void setCubeMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double scale_x, double scale_y, double scale_z, float transparency, const Color &color);
    static void setCylinderMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double height, double radius, float transparency, const Color &color);
    static void setSphereMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double radius, float transparency, const Color &color);

    static void setMarkerColor(visualization_msgs::Marker &marker, const Color &color, float transparency = 1.0);
    void setMarkerPoseAndTF(visualization_msgs::Marker &marker, const std::string &name, double x, double y, double yaw, double z);

    // ego and obstacle trajectory
    void add3DPointToEgoTrajectory(double x, double y, double t);
    void add3DPointToObsTrajectory(int obs_id, double x, double y, double t);
    void publishObsTrajectory(int obs_id, const std::string &topic_name, const Color &color);

    // coordinate system arrow
    void setArrowMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z, const Color &color);

    // obstacle information
    void setDirectionArrow(visualization_msgs::Marker &marker, const std::string &ns, int id, double x, double y, double z, double yaw, float transparency, const Color &color, double arrow_length = 3.5);
    void setEllipseMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double x, double y, double yaw, double ellipse_a, double ellipse_b, const Color &color);
    void setObstacleIntentText(visualization_msgs::Marker &marker, const std::string &ns, int id, double x, double y, const std::string &v, double yaw, float transparency, const std::string &intent, const std::string &probability);

    // road distance scale
    static void addDistanceScaleMarkers(visualization_msgs::MarkerArray &markers, const std::vector<double> &x, const std::vector<double> &y, const std::string &name, const Color &color);
    static void setTextMarker(visualization_msgs::Marker &marker, const std::string &name, int id, const std::string &text, double x, double y, double z, const Color &color);
    static void setScaleMarker(visualization_msgs::Marker &marker, const std::string &name, int id, double width, double height, const Color &color);

    // traffic light
    void publishTrafficLightTF(int id, const std::string &color, double x, double y, double z);
};