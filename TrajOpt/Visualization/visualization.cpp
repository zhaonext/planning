#include "visualization.h"
// Traffic light visualization
void Visualization::publishTrafficLightMarker(int id, double x, double y, double z, TrafficLightState state, TrafficLightOrientation orientation) {
    visualization_msgs::MarkerArray traffic_light_markers;

    visualization_msgs::Marker red_light, yellow_light, green_light;
    double light_radius = 1.0;                // 设置灯的半径
    double light_spacing = light_radius * 2;  // 设置灯之间的距离

    setSphereMarker(red_light, "traffic_light_red" + std::to_string(id), 0, light_radius, 1.0, RED);
    setSphereMarker(yellow_light, "traffic_light_yellow" + std::to_string(id), 1, light_radius, 1.0, YELLOW);
    setSphereMarker(green_light, "traffic_light_green" + std::to_string(id), 2, light_radius, 1.0, GREEN);

    // Set positions for horizontal arrangement
    if (orientation == TrafficLightOrientation::HORIZONTAL) {
        red_light.pose.position.x = x;
        red_light.pose.position.y = y;
        red_light.pose.position.z = z;

        yellow_light.pose.position.x = x + light_spacing;
        yellow_light.pose.position.y = y;
        yellow_light.pose.position.z = z;

        green_light.pose.position.x = x + 2 * light_spacing;
        green_light.pose.position.y = y;
        green_light.pose.position.z = z;
    } else {
        // Vertical arrangement
        red_light.pose.position.x = x;
        red_light.pose.position.y = y;
        red_light.pose.position.z = z;

        yellow_light.pose.position.x = x;
        yellow_light.pose.position.y = y + light_spacing;  // Yellow light above red
        yellow_light.pose.position.z = z;

        green_light.pose.position.x = x;
        green_light.pose.position.y = y + 2 * light_spacing;
        green_light.pose.position.z = z;
    }

    // Adjust transparency based on the current state
    switch (state) {
        case TrafficLightState::RED:
            red_light.color.a = 1.0;
            yellow_light.color.a = 0.1;
            green_light.color.a = 0.1;
            break;
        case TrafficLightState::YELLOW:
            red_light.color.a = 0.1;
            yellow_light.color.a = 1.0;
            green_light.color.a = 0.1;
            break;
        case TrafficLightState::GREEN:
            red_light.color.a = 0.1;
            yellow_light.color.a = 0.1;
            green_light.color.a = 1.0;
            break;
    }

    // 创建并发布每个交通灯的TF变换
    publishTrafficLightTF(id, "red", red_light.pose.position.x, red_light.pose.position.y, red_light.pose.position.z);
    publishTrafficLightTF(id, "yellow", yellow_light.pose.position.x, yellow_light.pose.position.y, yellow_light.pose.position.z);
    publishTrafficLightTF(id, "green", green_light.pose.position.x, green_light.pose.position.y, green_light.pose.position.z);

    // Add markers to the array
    traffic_light_markers.markers.push_back(red_light);
    traffic_light_markers.markers.push_back(yellow_light);
    traffic_light_markers.markers.push_back(green_light);

    // Publish the MarkerArray
    traffic_light_pub_.publish(traffic_light_markers);
}

// Zebra crossing publish: {marker},斑马线的宽度、条纹宽度和条纹间距: width, stripe_width, stripe_spacing
void Visualization::publishZebraCrossingMarker(int crossing_id, double start_x, double start_y, double length, double width, double stripe_width, double stripe_spacing, double yaw) {
    visualization_msgs::MarkerArray marker_array;

    for (double pos = 0; pos < length; pos += stripe_spacing + stripe_width) {
        visualization_msgs::Marker stripe;
        setCubeMarker(stripe, "zebra_crossing" + std::to_string(crossing_id), (int)marker_array.markers.size(), stripe_width, width, 0.01, 1.0, WHITE);

        // 计算每个条纹的位置
        double stripe_x = start_x + pos * cos(yaw);
        double stripe_y = start_y + pos * sin(yaw);

        // 设置条纹的位置和方向
        setMarkerPoseAndTF(stripe, "zebra_crossing_" + std::to_string(crossing_id), stripe_x, stripe_y, yaw, 0.01);

        marker_array.markers.push_back(stripe);
    }

    zebra_crossing_pub_.publish(marker_array);
}

// Road publish: {marker}
void Visualization::publishRoadMarkerWithScale(std::vector<double> &x_center, std::vector<double> &y_center,
                                               std::vector<double> &x_left, std::vector<double> &y_left,
                                               std::vector<double> &x_right, std::vector<double> &y_right,
                                               const std::string &topic_name, const Color &color) const {
    visualization_msgs::MarkerArray road_markers;
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

    // Center line with scales
    visualization_msgs::Marker center_line_marker;
    setDashedLineMarker(center_line_marker, topic_name + "_center", 0, 0.1, color);
    for (size_t i = 0; i < x_center.size(); ++i) {
        geometry_msgs::Point point;
        point.x = x_center.at(i);
        point.y = y_center.at(i);
        point.z = 0.0;
        center_line_marker.points.push_back(point);
    }
    //    road_markers.markers.push_back(center_line_marker);

    // Add distance scales to the center line
    //    addDistanceScaleMarkers(road_markers, x_center, y_center, topic_name + "_center_scale", color);

    // Left boundary
    visualization_msgs::Marker left_boundary_marker;
    setSolidLineMarker(left_boundary_marker, topic_name + "_left", 1, 0.1, color);
    for (size_t i = 0; i < x_left.size(); ++i) {
        geometry_msgs::Point point;
        point.x = x_left.at(i);
        point.y = y_left.at(i);
        point.z = 0.0;
        left_boundary_marker.points.push_back(point);
    }
    road_markers.markers.push_back(left_boundary_marker);

    // Right boundary
    visualization_msgs::Marker right_boundary_marker;
    setSolidLineMarker(right_boundary_marker, topic_name + "_right", 2, 0.1, color);
    for (size_t i = 0; i < x_right.size(); ++i) {
        geometry_msgs::Point point;
        point.x = x_right.at(i);
        point.y = y_right.at(i);
        point.z = 0.0;
        right_boundary_marker.points.push_back(point);
    }
    road_markers.markers.push_back(right_boundary_marker);

    // Publish the MarkerArray
    road_marker_array_pub_.publish(road_markers);
}

// ego marker publish: {marker + trajectory + coordinate_system}
void Visualization::publishEgoStateMarker(const TrajectoryPoint &state, const std::string &topic_name, const Color &color) {
    visualization_msgs::Marker vehicle_marker;
    ros::Time current_time = ros::Time::now();
    vehicle_marker.header.stamp = current_time;

    setCubeMarker(vehicle_marker, topic_name, 0, VEHICLE_LENGTH, VEHICLE_WIDTH, VEHICLE_HEIGHT, 0.9, color);
    setMarkerPoseAndTF(vehicle_marker, topic_name, state.x, state.y, state.phi, VEHICLE_HEIGHT / 2);
    add3DPointToEgoTrajectory(state.x, state.y, state.t);
    ego_state_pub_.publish(vehicle_marker);

    // Add coordinate axes
    double axis_length = 3.5;
    double end_x = state.x + axis_length * cos(state.phi);
    double end_y = state.y + axis_length * sin(state.phi);
    double perpendicular_end_x = state.x - axis_length * sin(state.phi);
    double perpendicular_end_y = state.y + axis_length * cos(state.phi);

    visualization_msgs::MarkerArray axes_array;
    visualization_msgs::Marker x_axis, y_axis, z_axis;
    x_axis.header.stamp = current_time;
    y_axis.header.stamp = current_time;
    z_axis.header.stamp = current_time;
    setArrowMarker(x_axis, "x_axis", 0, state.x, state.y, VEHICLE_HEIGHT / 2, end_x, end_y, VEHICLE_HEIGHT / 2, RED);
    setArrowMarker(y_axis, "y_axis", 1, state.x, state.y, VEHICLE_HEIGHT / 2, perpendicular_end_x, perpendicular_end_y, VEHICLE_HEIGHT / 2, GREEN);
    setArrowMarker(z_axis, "z_axis", 2, state.x, state.y, VEHICLE_HEIGHT / 2, state.x, state.y, VEHICLE_HEIGHT / 2 + axis_length, BLUE);
    axes_array.markers.push_back(x_axis);
    axes_array.markers.push_back(y_axis);
    axes_array.markers.push_back(z_axis);
    ego_axes_pub_.publish(axes_array);

    // publish 3D trajectory
    visualization_msgs::Marker trajectory_3d_marker;
    trajectory_3d_marker.header.stamp = current_time;
    setSolidLineMarker(trajectory_3d_marker, topic_name + "_3D", 0, 0.15, color);
    trajectory_3d_marker.points = ego_3d_trajectory_points_;
    trajectory_pub_.publish(trajectory_3d_marker);
}

// ego trajectory publish: {trajectory: 2D + 3D}
void Visualization::publishEgoTrajectory(const std::vector<TrajectoryPoint> &trajectory, const std::string &topic_name, const Color &color) const {
    if (trajectory.size() < 2) {
        return;
    }
    visualization_msgs::Marker trajectory_2d_marker;
    setSolidLineMarker(trajectory_2d_marker, topic_name + "_2D", 0, VEHICLE_WIDTH, color, 0.8);

    // 2D point
    std::vector<geometry_msgs::Point> ego_2d_trajectory_points_;
    for (const auto &point : trajectory) {
        geometry_msgs::Point xy_point;
        xy_point.x = point.x;
        xy_point.y = point.y;
        ego_2d_trajectory_points_.push_back(xy_point);
    }

    // publish 2D trajectory
    trajectory_2d_marker.points = ego_2d_trajectory_points_;
    trajectory_pub_.publish(trajectory_2d_marker);
}
void Visualization::publishEgoReferenceLine(const std::vector<Vec2d> &ref_line, const std::string &topic_name, const Color &color) {
    auto line = ref_line;
    if (line.size() < 2) {
        return;
    }
    if (line.size() % 2 != 0) {
        line.pop_back();
    }
    visualization_msgs::Marker trajectory_2d_marker;
    setSolidLineMarker(trajectory_2d_marker, topic_name + "_2D", 0, LANE_WIDTH * 0.1, color, 0.8);

    // 2D point
    std::vector<geometry_msgs::Point> reference_line;
    for (const auto &point : line) {
        geometry_msgs::Point xy_point;
        xy_point.x = point.x();
        xy_point.y = point.y();
        xy_point.z = -0.1;
        reference_line.push_back(xy_point);
    }

    // publish 2D trajectory
    trajectory_2d_marker.points = reference_line;
    trajectory_pub_.publish(trajectory_2d_marker);
}

// target obstacle marker publish: {marker + id + intent + probability + ellipse}
void Visualization::publishTargetObstacleMarker(const TrajectoryPoint &state, const int id, ObstacleType obs_type, ObstacleIntent obs_intent,
                                                const double obs_probability, const std::string &topic_name, const std::pair<double, double> &ellipse_param, float transparency) {
    const Color color = getColorByObstacleType(obs_type);
    const std::string intent = getObstacleIntentString(obs_intent);
    const std::string probability = changeDoubleFormat(obs_probability);
    const std::string velocity = changeDoubleFormat(state.v * 3.6);

    visualization_msgs::MarkerArray marker_array;
    ros::Time current_time = ros::Time::now();

    // Create marker
    visualization_msgs::Marker marker;
    marker.header.stamp = current_time;
    add3DPointToObsTrajectory(id, state.x, state.y, state.t);
    // Direction arrow
    visualization_msgs::Marker direction_arrow;
    direction_arrow.header.stamp = current_time;
    if (obs_type == ObstacleType::DYNAMIC_VEHICLE) {
        setCubeMarker(marker, topic_name + std::to_string(id), id, VEHICLE_LENGTH, VEHICLE_WIDTH, VEHICLE_HEIGHT, transparency, color);
        setMarkerPoseAndTF(marker, topic_name, state.x, state.y, state.phi, VEHICLE_HEIGHT / 2);
        setDirectionArrow(direction_arrow, topic_name + "_direction" + std::to_string(id), id, state.x, state.y, VEHICLE_HEIGHT / 2, state.phi, transparency, color);
    } else if (obs_type == ObstacleType::PEDESTRIAN) {
        double PEDESTRIAN_HEIGHT = 1.75;  // 假设行人身高1.75m
        setCylinderMarker(marker, topic_name + std::to_string(id), id, PEDESTRIAN_HEIGHT, 0.35, transparency, color);
        setMarkerPoseAndTF(marker, topic_name + std::to_string(id), state.x, state.y, state.phi, PEDESTRIAN_HEIGHT / 2);
        setDirectionArrow(direction_arrow, topic_name + "_direction" + std::to_string(id), id, state.x, state.y, PEDESTRIAN_HEIGHT / 2, state.phi, transparency, color, 2.0);
    }

    // Intent + velocity + Probability text
    visualization_msgs::Marker intent_probability_text_marker;
    intent_probability_text_marker.header.stamp = current_time;
    setObstacleIntentText(intent_probability_text_marker, topic_name + "_intent_probability" + std::to_string(id), id, state.x, state.y, velocity, state.phi, transparency, intent, probability);

    // envelope_polygon
    visualization_msgs::Marker ellipse_marker;
    ellipse_marker.header.stamp = current_time;
    auto ellipse_a = ellipse_param.first;
    auto ellipse_b = ellipse_param.second;
    setEllipseMarker(ellipse_marker, topic_name + "_ellipse" + std::to_string(id), id, state.x, state.y, state.phi, ellipse_a, ellipse_b, color);

    // 设置Marker的生命周期为0.1秒
    //        ros::Duration marker_lifetime(0.5);
    //        marker.lifetime = marker_lifetime;
    //        direction_arrow.lifetime = marker_lifetime;
    //        ellipse_marker.lifetime = marker_lifetime;
    //        intent_probability_text_marker.lifetime = marker_lifetime;

    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(direction_arrow);
    marker_array.markers.push_back(ellipse_marker);
    marker_array.markers.push_back(intent_probability_text_marker);

    target_obs_state_pub_.publish(marker_array);
    publishObsTrajectory(id, topic_name, color);
}
// obs trajectory publish: {trajectory: 2D + 3D}
void Visualization::publishAllObstacleTrajectory(const std::vector<TrajectoryPoint> &trajectory, int id, const std::string &topic_name, const Color &color, float transparency) const {
    if (trajectory.size() < 2) {
        return;
    }
    visualization_msgs::Marker trajectory_2d_marker;
    setSolidLineMarker(trajectory_2d_marker, topic_name + "_2D", id, 0.3, color, transparency);

    // 2D point
    std::vector<geometry_msgs::Point> obs_2d_trajectory_points_;
    for (const auto &point : trajectory) {
        geometry_msgs::Point xy_point;
        xy_point.x = point.x;
        xy_point.y = point.y;
        obs_2d_trajectory_points_.push_back(xy_point);
    }

    // publish 2D trajectory
    trajectory_2d_marker.points = obs_2d_trajectory_points_;
    trajectory_pub_.publish(trajectory_2d_marker);
}

void Visualization::publishNoTargetObstacleMarker(const TrajectoryPoint &state, const int id, ObstacleType obs_type, ObstacleIntent obs_intent,
                                                  const double obs_probability, const std::string &topic_name, const std::pair<double, double> &ellipse_param, float transparency) {
    const Color color = getColorByObstacleType(obs_type);

    visualization_msgs::MarkerArray marker_array;
    ros::Time current_time = ros::Time::now();

    // Create marker
    visualization_msgs::Marker marker;
    marker.header.stamp = current_time;
    add3DPointToObsTrajectory(id, state.x, state.y, state.t);
    // Direction arrow
    visualization_msgs::Marker direction_arrow;
    direction_arrow.header.stamp = current_time;
    if (obs_type == ObstacleType::DYNAMIC_VEHICLE) {
        setCubeMarker(marker, "no_" + topic_name + std::to_string(id), id, VEHICLE_LENGTH, VEHICLE_WIDTH, VEHICLE_HEIGHT, transparency, color);
        setMarkerPoseAndTF(marker, "no_" + topic_name, state.x, state.y, state.phi, VEHICLE_HEIGHT / 2);
        setDirectionArrow(direction_arrow, "no_" + topic_name + "_direction" + std::to_string(id), id, state.x, state.y, VEHICLE_HEIGHT / 2, state.phi, transparency, color);
    } else if (obs_type == ObstacleType::PEDESTRIAN) {
        double PEDESTRIAN_HEIGHT = 1.75;  // 假设行人身高1.75m
        setCylinderMarker(marker, "no_" + topic_name + std::to_string(id), id, PEDESTRIAN_HEIGHT, 0.35, transparency, color);
        setMarkerPoseAndTF(marker, "no_" + topic_name + std::to_string(id), state.x, state.y, state.phi, PEDESTRIAN_HEIGHT / 2);
        setDirectionArrow(direction_arrow, "no_" + topic_name + "_direction" + std::to_string(id), id, state.x, state.y, PEDESTRIAN_HEIGHT / 2, state.phi, transparency, color, 2.0);
    }

    // 设置Marker的生命周期为0.1秒
    ros::Duration marker_lifetime(0.5);
    marker.lifetime = marker_lifetime;
    direction_arrow.lifetime = marker_lifetime;

    marker_array.markers.push_back(marker);
    marker_array.markers.push_back(direction_arrow);

    no_target_obs_state_pub_.publish(marker_array);
}

/* dashboard */
// 1.speed + steering_angle
void Visualization::publishEgoSpeedAndSteeringAngleAndAcceleration(double speed, double steering_angle, double acceleration) {
    std_msgs::Float32 speed_msg;
    speed_msg.data = (float)speed;
    speed_pub_.publish(speed_msg);

    std_msgs::Float32 steering_angle_msg;
    steering_angle_msg.data = (float)steering_angle;
    steering_angle_pub_.publish(steering_angle_msg);

    std_msgs::Float32 acceleration_msg;
    acceleration_msg.data = (float)acceleration;
    acceleration_pub_.publish(acceleration_msg);
}

// 2.time + Gear
void Visualization::publishTimeAndGearOverlay(const std::string &time, const std::string &gear) {
    jsk_rviz_plugins::OverlayText text;
    text.font = "DejaVu Sans Mono";
    text.text = "Gear: " + gear + "\n\n" + "Time: " + time + "s";
    time_gear_pub_.publish(text);
}

// global coordinate system
void Visualization::publishStaticAxes() {
    double axis_length = 15;
    visualization_msgs::Marker x_axis, y_axis, z_axis;
    visualization_msgs::MarkerArray axes_array;
    setArrowMarker(x_axis, "global_axes", 0, 0, 0, 0, axis_length, 0, 0, RED);
    axes_array.markers.push_back(x_axis);
    setArrowMarker(y_axis, "global_axes", 1, 0, 0, 0, 0, axis_length, 0, GREEN);
    axes_array.markers.push_back(y_axis);
    setArrowMarker(z_axis, "global_axes", 2, 0, 0, 0, 0, 0, axis_length, BLUE);
    axes_array.markers.push_back(z_axis);
    global_axes_pub_.publish(axes_array);
}

void Visualization::initPublishers() {
    route_pub_ = nh_.advertise<visualization_msgs::Marker>("/route_marker", 10);
    waypoints_pub_ = nh_.advertise<visualization_msgs::Marker>("/waypoints_marker", 10);
    ego_state_pub_ = nh_.advertise<visualization_msgs::Marker>("/ego_state_marker", 1);
    target_obs_state_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/target_obs_state_marker", 1);
    no_target_obs_state_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/no_target_obs_state_marker", 1);
    trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("/vehicle_trajectory", 10);
    speed_pub_ = nh_.advertise<std_msgs::Float32>("/ego_speed", 10);
    steering_angle_pub_ = nh_.advertise<std_msgs::Float32>("/ego_steering_angle", 10);
    acceleration_pub_ = nh_.advertise<std_msgs::Float32>("/ego_acceleration", 10);
    time_gear_pub_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("/time_gear", 1);
    global_axes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_axes", 1);
    ego_axes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ego_axes", 1);
    road_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/road_marker", 10);
    zebra_crossing_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/zebra_crossing", 10);
    traffic_light_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/traffic_light", 10);
    stop_line_pub_ = nh_.advertise<visualization_msgs::Marker>("/stop_line", 1);
    speed_profile_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/speed_profile", 10);
}

void Visualization::setDashedLineMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double scale, const Color &color, float transparency) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    setMarkerColor(marker, color);
}
void Visualization::setSolidLineMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double scale, const Color &color, float transparency) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    setMarkerColor(marker, color, transparency);
}
void Visualization::setCubeMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double scale_x, double scale_y, double scale_z, float transparency, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    setMarkerColor(marker, color, transparency);
}
void Visualization::setCylinderMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double height, double radius, float transparency, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius * 2;  // Diameter in X
    marker.scale.y = radius * 2;  // Diameter in Y
    marker.scale.z = height;      // Height of the cylinder
    setMarkerColor(marker, color, transparency);
}
void Visualization::setSphereMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double radius, float transparency, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius * 2;  // Diameter in X
    marker.scale.y = radius * 2;  // Diameter in Y
    marker.scale.z = radius * 2;  // Diameter in Z to make it a sphere
    setMarkerColor(marker, color, transparency);
}

void Visualization::setMarkerColor(visualization_msgs::Marker &marker, const Color &color, float transparency) {
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = transparency;
}
void Visualization::setMarkerPoseAndTF(visualization_msgs::Marker &marker, const std::string &name, double x, double y, double yaw, double z) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);

    // 设置标记的方向
    marker.pose.orientation = tf2::toMsg(orientation);

    // 发布TF变换
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = name + "_base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation = tf2::toMsg(orientation);

    tf_broadcaster_.sendTransform(transformStamped);
}

void Visualization::add3DPointToEgoTrajectory(double x, double y, double t) {
    // 3D point
    geometry_msgs::Point xyt_point;
    xyt_point.x = x;
    xyt_point.y = y;
    xyt_point.z = t;
    ego_3d_trajectory_points_.push_back(xyt_point);
}
void Visualization::add3DPointToObsTrajectory(int obs_id, double x, double y, double t) {
    // 3D point
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = t;
    obs_trajectories_[obs_id].push_back(point);
}
void Visualization::publishObsTrajectory(int obs_id, const std::string &topic_name, const Color &color) {
    visualization_msgs::Marker trajectory_marker;
    setSolidLineMarker(trajectory_marker, topic_name, obs_id, 0.15, color);
    trajectory_marker.points = obs_trajectories_[obs_id];
    trajectory_pub_.publish(trajectory_marker);
}

void Visualization::setArrowMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;  // Set unique ID for each marker
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.5;
    setMarkerColor(marker, color);

    geometry_msgs::Point start, end;
    start.x = start_x;
    start.y = start_y;
    start.z = start_z;
    end.x = end_x;
    end.y = end_y;
    end.z = end_z;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // Publish the TF transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ns + "_" + std::to_string(id) + "_base_link";
    transformStamped.transform.translation.x = start_x;
    transformStamped.transform.translation.y = start_y;
    transformStamped.transform.translation.z = start_z;

    // Calculate the orientation from the arrow direction
    tf2::Quaternion orientation;
    tf2::Vector3 direction(end_x - start_x, end_y - start_y, end_z - start_z);
    direction.normalize();
    orientation.setRPY(0, 0, std::atan2(direction.y(), direction.x()));
    transformStamped.transform.rotation.x = orientation.x();
    transformStamped.transform.rotation.y = orientation.y();
    transformStamped.transform.rotation.z = orientation.z();
    transformStamped.transform.rotation.w = orientation.w();

    tf_broadcaster_.sendTransform(transformStamped);
}

void Visualization::setDirectionArrow(visualization_msgs::Marker &marker, const std::string &ns, int id, double x, double y, double z, double yaw, float transparency, const Color &color, double arrow_length) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;  // 使用obs_id作为标记ID
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.5;
    setMarkerColor(marker, color, transparency);

    geometry_msgs::Point start, end;
    start.x = x;
    start.y = y;
    start.z = z;  // 调整高度以使箭头出现在障碍物上方
    end.x = x + arrow_length * cos(yaw);
    end.y = y + arrow_length * sin(yaw);
    end.z = z;  // 调整高度以使箭头出现在障碍物上方

    marker.points.push_back(start);
    marker.points.push_back(end);

    // Publish the TF transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ns + "_" + std::to_string(id) + "_base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.5;  // Assuming the obstacle's height is centered at 0.5

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = orientation.x();
    transformStamped.transform.rotation.y = orientation.y();
    transformStamped.transform.rotation.z = orientation.z();
    transformStamped.transform.rotation.w = orientation.w();

    tf_broadcaster_.sendTransform(transformStamped);
}
void Visualization::setEllipseMarker(visualization_msgs::Marker &marker, const std::string &ns, int id, double x, double y, double yaw, double ellipse_a, double ellipse_b, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;  // 使用obs_id作为标记ID
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;  // 将椭圆放置在障碍物的中心高度

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = std::max(ellipse_a, 0.1) * 2;  // 椭圆的长轴
    marker.scale.y = std::max(ellipse_b, 0.1) * 2;  // 椭圆的短轴
    marker.scale.z = 0.01;                          // 椭圆的高度，设置为很小的值使其看起来像一个椭圆

    setMarkerColor(marker, color);
    marker.color.a = 0.5;  // 设置为半透明

    // Publish the TF transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ns + "_" + std::to_string(id) + "_base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0;  // Assuming the ellipse's height is centered at 0
    transformStamped.transform.rotation = marker.pose.orientation;

    tf_broadcaster_.sendTransform(transformStamped);
}
void Visualization::setObstacleIntentText(visualization_msgs::Marker &marker, const std::string &ns, const int id, const double x, const double y, const std::string &v, double yaw, float transparency, const std::string &intent, const std::string &probability) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = VEHICLE_HEIGHT + 1.0;  // 调整高度以使文本出现在障碍物上方
    marker.pose.orientation.w = 1.0;
    marker.text = "ID: " + std::to_string(id) + ", " + intent + ": " + probability + "\n" + v + " km/h";
    marker.scale.z = 0.8;  // 调整文本大小
    setMarkerColor(marker, WHITE, transparency);

    // Publish the TF transform
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ns + "_" + std::to_string(id) + "_base_link";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = VEHICLE_HEIGHT;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = orientation.x();
    transformStamped.transform.rotation.y = orientation.y();
    transformStamped.transform.rotation.z = orientation.z();
    transformStamped.transform.rotation.w = orientation.w();

    tf_broadcaster_.sendTransform(transformStamped);
}

void Visualization::addDistanceScaleMarkers(visualization_msgs::MarkerArray &markers, const std::vector<double> &x, const std::vector<double> &y, const std::string &name, const Color &color) {
    double accumulated_distance = 0.0;
    double scale = 20.0;
    double next_scale_distance = scale;

    for (size_t i = 1; i < x.size(); ++i) {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        double segment_length = std::sqrt(dx * dx + dy * dy);
        accumulated_distance += segment_length;

        if (accumulated_distance >= next_scale_distance) {
            double alpha = (next_scale_distance - (accumulated_distance - segment_length)) / segment_length;

            double marker_x = x[i - 1] + alpha * dx;
            double marker_y = y[i - 1] + alpha * dy;

            visualization_msgs::Marker scale_marker;
            setScaleMarker(scale_marker, name, (int)i, 0.2, 0.5, color);
            scale_marker.pose.position.x = marker_x;
            scale_marker.pose.position.y = marker_y;
            scale_marker.pose.position.z = 0.0;
            scale_marker.pose.orientation.w = 1.0;
            markers.markers.push_back(scale_marker);

            // Add text marker for distance
            visualization_msgs::Marker text_marker;
            setTextMarker(text_marker, name + "_text", (int)i, std::to_string(static_cast<int>(next_scale_distance)) + "m", marker_x, marker_y, 0.5, color);
            markers.markers.push_back(text_marker);
            next_scale_distance += scale;
        }
    }
}
void Visualization::setTextMarker(visualization_msgs::Marker &marker, const std::string &name, int id, const std::string &text, double x, double y, double z, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.text = text;
    marker.scale.z = 0.5;  // Text size
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;
}
void Visualization::setScaleMarker(visualization_msgs::Marker &marker, const std::string &name, int id, double width, double height, const Color &color) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = width;
    marker.scale.y = height;
    marker.scale.z = 0.01;  // very thin in z direction
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;
}

void Visualization::publishStopLineMarker(bool reach_stop_line, double x, double y, double yaw, double length, double width, double height, const Color &color) {
    visualization_msgs::Marker stop_line_marker;
    if (reach_stop_line) {
        // 红灯 - 设置停止线标记
        std::string ns = "stop_line";
        int id = 0;
        setCubeMarker(stop_line_marker, ns, id, length, width, height, 0.7, color);
        setMarkerPoseAndTF(stop_line_marker, ns, x, y, yaw, height / 2);
        stop_line_marker.action = visualization_msgs::Marker::ADD;
    } else {
        // 不是红灯 - 设置清除标记
        stop_line_marker.ns = "stop_line";
        stop_line_marker.id = 0;
        stop_line_marker.action = visualization_msgs::Marker::DELETE;
    }

    stop_line_pub_.publish(stop_line_marker);
}

void Visualization::publishTrafficLightTF(int id, const std::string &color, double x, double y, double z) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "traffic_light_" + std::to_string(id);
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_.sendTransform(transformStamped);
}

void Visualization::publishSpeedProfile(const std::vector<std::tuple<double, double, double>> &speed_profile, const std::string &name, int id, const Color &color) {
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < speed_profile.size(); i += 2) {
        auto [x, y, v] = speed_profile[i];
        visualization_msgs::Marker marker;
        setCylinderMarker(marker, name, (int)(id + i), v, 0.05, 0.7, color);
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = v / 2;
        marker_array.markers.push_back(marker);
    }
    speed_profile_pub_.publish(marker_array);
}
