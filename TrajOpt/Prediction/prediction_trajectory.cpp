#include "prediction_trajectory.h"

bool ObstaclesManager::updateObstacleState(const State& ego_state, const int index) {
    getNextFramePredictedTrajectory(ego_state, index);
    updateObstacleInfo();
    //        UpdateAllObstacleWithIDM(index, time_step_);

    return true;
}

void ObstaclesManager::addObstacleToList(const ObstacleInfo& obstacle_info) {
    obstacle_list_.emplace_back(obstacle_info);
    calAllObstaclePredictedTrajectory();
}

std::vector<std::pair<ObstacleType, TrajectoryPoint>> ObstaclesManager::getTargetObstacleInfosAtIndex(const int index) const {
    std::vector<std::pair<ObstacleType, TrajectoryPoint>> traj_points;
    traj_points.reserve(target_obstacle_prediction_trajectories_.size());

    for (const auto& obstacleTrajectory : target_obstacle_prediction_trajectories_) {
        double max_probability = -INF;
        Trajectory max_prob_trajectory;

        // 遍历每个障碍物的多模态轨迹，找到概率最大的轨迹
        for (const auto& intentTrajectoryPair : obstacleTrajectory.multimode_trajectories) {
            double probability = intentTrajectoryPair.second.first;  // 第一个元素是概率
            if (probability > max_probability) {
                max_probability = probability;
                max_prob_trajectory = intentTrajectoryPair.second.second;  // 第二个元素是轨迹
            }
        }

        // 检查是否找到了有效的轨迹，以及索引是否在合法范围内
        if (max_probability < 0.0 || index < 0 || index >= max_prob_trajectory.points.size()) {
            AERROR << "Error: No valid trajectory found or index out of bounds for obstacle ID " << obstacleTrajectory.id;
            continue;
        }
        traj_points.emplace_back(obstacleTrajectory.type, max_prob_trajectory.points[index]);
    }
    return traj_points;
}

void ObstaclesManager::updateObstacleInfo() {
    obstacle_list_.clear();
    AINFO << "Updating obstacle list. Target size: " << target_obstacle_init_state_.size()
          << ", Non-target size: " << no_target_obstacle_init_state_.size();

    // 输出并添加目标障碍物
    for (const auto& targetState : target_obstacle_init_state_) {
        AINFO << "Target obstacle ID " << targetState.id << " added with priority "
              << targetState.priority;
        obstacle_list_.push_back(targetState);
    }

    // 输出并添加非目标障碍物
    for (const auto& noTargetState : no_target_obstacle_init_state_) {
        //            AINFO << "Non-target obstacle ID " << noTargetState.id << " added with priority "
        //                  << noTargetState.priority;
        obstacle_list_.push_back(noTargetState);
    }

    all_obstacle_prediction_trajectories_.clear();
    // 将目标障碍物的预测轨迹添加到全部预测轨迹列表
    all_obstacle_prediction_trajectories_.insert(
        all_obstacle_prediction_trajectories_.end(),
        target_obstacle_prediction_trajectories_.begin(),
        target_obstacle_prediction_trajectories_.end());

    // 将非目标障碍物的预测轨迹添加到全部预测轨迹列表
    all_obstacle_prediction_trajectories_.insert(
        all_obstacle_prediction_trajectories_.end(),
        no_target_obstacle_prediction_trajectories_.begin(),
        no_target_obstacle_prediction_trajectories_.end());
}

bool ObstaclesManager::calAllObstaclePredictedTrajectory() {
    all_obstacle_prediction_trajectories_.clear();
    all_obstacle_prediction_trajectories_.reserve(obstacle_list_.size());

    bool all_success = true;
    for (const auto& obsInfo : obstacle_list_) {
        PredictionTrajectory predictionTrajectory(obsInfo);
        bool success_flag = predictionTrajectory.predictTrajectory(horizon_, time_step_);
        if (!success_flag) {
            AERROR << "Error: Failed to predict trajectory for obstacle with ID " << obsInfo.id;
            all_success = false;
            continue;
        }
        auto trajectory = predictionTrajectory.GetTrajectory();
        // TODO:: update obs prediction probability
        double probability = calculateProbability(obsInfo);
        ObstacleTrajectory obsTraj{obsInfo.id, obsInfo.type, {{obsInfo.intent, {probability, trajectory}}}};
        all_obstacle_prediction_trajectories_.emplace_back(std::move(obsTraj));
    }
    return all_success;
}

bool ObstaclesManager::getNextFramePredictedTrajectory(const State& ego_state, const int index) {
    target_obstacle_init_state_.clear();
    no_target_obstacle_init_state_.clear();
    target_obstacle_prediction_trajectories_.clear();
    no_target_obstacle_prediction_trajectories_.clear();

    // 遍历所有障碍物的预测轨迹
    for (const auto& trajectory : all_obstacle_prediction_trajectories_) {
        double max_probability = -INF;
        Trajectory max_prob_trajectory;
        ObstacleIntent max_prob_intent;

        // 遍历障碍物的多模态轨迹，找出概率最大的轨迹
        for (const auto& intentTrajectoryPair : trajectory.multimode_trajectories) {
            double probability = intentTrajectoryPair.second.first;  // 概率值
            if (probability > max_probability) {
                max_probability = probability;
                max_prob_trajectory = intentTrajectoryPair.second.second;  // 对应的轨迹
                max_prob_intent = intentTrajectoryPair.first;              // 对应的意图
            }
        }

        // 检查索引是否有效
        if (max_probability < 0.0 || index < 0 || index >= max_prob_trajectory.points.size()) {
            AERROR << "Error: No valid intent found or index out of bounds for obstacle ID " << trajectory.id;
            continue;
        }

        // 提取对应索引处的轨迹点作为下一帧的初始状态
        const auto& next_frame_init_point = max_prob_trajectory.points[index];

        ObstacleInfo updatedInfo;
        updatedInfo.id = trajectory.id;
        updatedInfo.type = trajectory.type;
        updatedInfo.intent = max_prob_intent;
        updatedInfo.priority = max_probability;  // 保存最大概率作为障碍物的优先级
        updatedInfo.x = next_frame_init_point.x;
        updatedInfo.y = next_frame_init_point.y;
        updatedInfo.phi = next_frame_init_point.phi;
        updatedInfo.v = next_frame_init_point.v;
        updatedInfo.t = next_frame_init_point.t;

        // 根据障碍物的初始状态生成新的预测轨迹
        PredictionTrajectory predictionTrajectory(updatedInfo);
        if (!predictionTrajectory.predictTrajectory(horizon_, time_step_)) {
            AERROR << "Error: Failed to predict trajectory for obstacle with ID " << updatedInfo.id;
            continue;
        }
        auto predicted_trajectory = predictionTrajectory.GetTrajectory();
        // TODO::重新计算概率
        double probability = calculateProbability(updatedInfo);

        ObstacleTrajectory obsTraj{updatedInfo.id, updatedInfo.type, {{updatedInfo.intent, {probability, predicted_trajectory}}}};

        // 根据障碍物是否为目标障碍物，分类并保存初始状态和预测轨迹
        if (isTargetObstacle(ego_state, next_frame_init_point)) {
            target_obstacle_prediction_trajectories_.push_back(obsTraj);
            target_obstacle_init_state_.push_back(updatedInfo);
        } else {
            no_target_obstacle_prediction_trajectories_.push_back(obsTraj);
            no_target_obstacle_init_state_.push_back(updatedInfo);
        }
    }
    // 如果任一列表不为空，则返回true
    return !target_obstacle_init_state_.empty() || !no_target_obstacle_init_state_.empty();
}

 bool ObstaclesManager::isTargetObstacle(const State& ego_state, const TrajectoryPoint& obstacle_point) {
    constexpr double FRONT_DISTANCE_THRESHOLD = 100.0;                      // 前方距离阈值
    constexpr double REAR_DISTANCE_THRESHOLD = 20.0;                        // 后方距离阈值
    const double SIDE_DISTANCE_THRESHOLD = LANE_WIDTH + VEHICLE_WIDTH / 2;  // 侧向距离阈值

    const double dx = obstacle_point.x - ego_state.x;
    const double dy = obstacle_point.y - ego_state.y;

    // 转换到自车坐标系
    const double x_relative = dx * cos(ego_state.phi) + dy * sin(ego_state.phi);
    const double y_relative = -dx * sin(ego_state.phi) + dy * cos(ego_state.phi);

    // x方向筛选
    const bool in_front = x_relative > 0 && x_relative <= FRONT_DISTANCE_THRESHOLD;
    const bool behind = x_relative < 0 && -x_relative <= REAR_DISTANCE_THRESHOLD;

    // y方向筛选
    bool on_side = std::abs(y_relative) <= SIDE_DISTANCE_THRESHOLD;

    return (in_front || behind) && on_side;
}

double ObstaclesManager::calculateProbability(const ObstacleInfo& obsInfo) {
    // TODO:: Add obstacles to the probability of each intention
    return 0.5;
}

void ObstaclesManager::UpdateAllObstacleWithIDM(const int index, const double dt) {
    for (auto& obstacle : obstacle_list_) {
        auto [x_front, y_front, v_front, phi_front] = FindLeadingVehicle(obstacle);
        const auto [acceleration, new_velocity] = UpdateObstacleWithIDM(obstacle, x_front, y_front, v_front, phi_front, index * dt);

        // 更新障碍物的速度和加速度
        obstacle.v = new_velocity;
        obstacle.a = acceleration;

        // 更新对应障碍物轨迹中的速度和加速度信息
        auto traj_it = std::find_if(all_obstacle_prediction_trajectories_.begin(), all_obstacle_prediction_trajectories_.end(),
                                    [&](const ObstacleTrajectory& traj) { return traj.id == obstacle.id; });

        if (traj_it != all_obstacle_prediction_trajectories_.end()) {
            for (auto& [intent, traj_pair] : traj_it->multimode_trajectories) {
                if (index < traj_pair.second.points.size()) {
                    // 更新指定轨迹点的速度和加速度
                    traj_pair.second.points[index].v = new_velocity;
                }
            }
        }
    }
}

// 查找前车并计算纵向距离和速度
std::tuple<double, double, double, double> ObstaclesManager::FindLeadingVehicle(const ObstacleInfo& obstacle) const {
    double x_front = INF;
    double y_front = INF;
    double v_front = 0.0;
    double phi_front = 0.0;

    // 遍历障碍物列表，寻找前车
    for (const auto& other_obstacle : obstacle_list_) {
        if (other_obstacle.id != obstacle.id) {
            // 计算相对位置
            const double delta_x = other_obstacle.x - obstacle.x;
            const double delta_y = other_obstacle.y - obstacle.y;

            // 转换到当前障碍物的局部坐标系
            const double x_relative = delta_x * std::cos(obstacle.phi) + delta_y * std::sin(obstacle.phi);
            const double y_relative = -delta_x * std::sin(obstacle.phi) + delta_y * std::cos(obstacle.phi);

            // 检查是否在前方且在车道内
            if (x_relative > 0 && std::abs(y_relative) < MAXIMUM_LATERAL_OFFSET / 2) {
                // 如果这个障碍物比已知的前车更近，则更新前车信息
                if (x_relative < x_front) {
                    x_front = x_relative;
                    y_front = y_relative;
                    v_front = other_obstacle.v;
                    phi_front = other_obstacle.phi;
                }
            }
        }
    }

    return {x_front, y_front, v_front, phi_front};
}

std::pair<double, double> ObstaclesManager::UpdateObstacleWithIDM(const ObstacleInfo& obstacle, const double x_front, const double y_front, const double v_front, const double phi_front, const double dt) const {
    const IntelligentDriverModel::IDMState idm_state{obstacle.x, obstacle.y, obstacle.phi, obstacle.v, x_front, y_front, phi_front, v_front};

    // 使用IDM模型计算加速度和新速度
    const auto [acceleration, new_velocity] = IntelligentDriverModel::calIdmDesiredAccelerationAndVelocity(idm_params_, idm_state, dt);
    return {acceleration, new_velocity};
}

