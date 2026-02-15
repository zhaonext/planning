#pragma once
#include "geometry_msgs/PoseStamped.h"
#include "intelligent_driver_model.h"
#include "prediction_model.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

class PredictionTrajectory {
public:
    explicit PredictionTrajectory(const ObstacleInfo& info) : ctrv(info) {}

    bool predictTrajectory(const double horizon, const double time_step) {
        trajectory = ctrv.predict_trajectory(horizon, time_step);
        return !trajectory.points.empty();
    }

    const Trajectory& GetTrajectory() const { return trajectory; }

private:
    ObstacleVehiclePredictiveModel ctrv;
    Trajectory trajectory;
};

class ObstaclesManager {
public:
    ObstaclesManager(const std::vector<ObstacleInfo>& obstacles, const int horizon, const double time_step)
        : obstacle_list_(obstacles), horizon_(horizon), time_step_(time_step) {
        calAllObstaclePredictedTrajectory();
    }

    /**
     * @brief 更新障碍物状态，并重新计算预测轨迹
     * @param ego_state
     * @param index
     * @return
     */
    bool updateObstacleState(const State& ego_state, const int index);

    /**
     * @brief  向障碍物列表中添加一个新障碍物，并更新预测轨迹
     * @param obstacle_info
     */
    void addObstacleToList(const ObstacleInfo& obstacle_info);

    /**
     * @brief 根据索引获取目标障碍物的预测轨迹点信息
     * @param index
     * @return
     */
    std::vector<std::pair<ObstacleType, TrajectoryPoint>> getTargetObstacleInfosAtIndex(int index) const;

    const std::vector<ObstacleInfo>& GetTargetObstacleInitState() const { return target_obstacle_init_state_; }
    const std::vector<ObstacleInfo>& GetNoTargetObstacleInitState() const { return no_target_obstacle_init_state_; }
    const std::vector<ObstacleTrajectory>& GetAllObstaclePredictionTrajectories() const { return all_obstacle_prediction_trajectories_; }
    const vector<ObstacleTrajectory>& GetTargetObstaclePredictionTrajectories() const { return target_obstacle_prediction_trajectories_; }
    const vector<ObstacleTrajectory>& GetNoTargetObstaclePredictionTrajectories() const { return no_target_obstacle_prediction_trajectories_; }

private:
    /**
     * @brief update obstacle_list_ and all_obstacle_prediction_trajectories_
     */
    void updateObstacleInfo();

    /**
     * @brief 计算所有障碍物的预测轨迹
     * @return
     */
    bool calAllObstaclePredictedTrajectory();

    /**
     * @brief 获取下一帧的预测轨迹，更新目标和非目标障碍物的初始状态和预测轨迹
     * @param ego_state
     * @param index
     * @return
     */
    bool getNextFramePredictedTrajectory(const State& ego_state, const int index);

    static bool isTargetObstacle(const State& ego_state, const TrajectoryPoint& obstacle_point);

    static double calculateProbability(const ObstacleInfo& obsInfo);

    /**
     * @brief 用于更新所有障碍物的速度和加速度
     * @param dt
     */
    void UpdateAllObstacleWithIDM(int index, double dt);

private:
    // 查找前车并计算纵向距离和速度
    std::tuple<double, double, double, double> FindLeadingVehicle(const ObstacleInfo& obstacle) const;

    /**
     * @brief 更新单个障碍物的速度和加速度
     * @param obstacle
     * @param x_front
     * @param y_front
     * @param v_front
     * @param phi_front
     * @param dt
     */
    std::pair<double, double> UpdateObstacleWithIDM(const ObstacleInfo& obstacle, const double x_front, const double y_front, const double v_front, const double phi_front, const double dt) const;

    int horizon_;
    double time_step_;
    std::vector<ObstacleInfo> obstacle_list_;
    std::vector<ObstacleTrajectory> all_obstacle_prediction_trajectories_;

private:
    std::vector<ObstacleInfo> target_obstacle_init_state_;
    std::vector<ObstacleInfo> no_target_obstacle_init_state_;

    std::vector<ObstacleTrajectory> target_obstacle_prediction_trajectories_;

public:
    const vector<ObstacleTrajectory>& getTargetObstaclePredictionTrajectories() const;
    const vector<ObstacleTrajectory>& getNoTargetObstaclePredictionTrajectories() const;

private:
    // 目标障碍物的预测轨迹
    std::vector<ObstacleTrajectory> no_target_obstacle_prediction_trajectories_;  // 非目标障碍物的预测轨迹

    IntelligentDriverModel::IDMParam idm_params_;  // IDM模型参数
};