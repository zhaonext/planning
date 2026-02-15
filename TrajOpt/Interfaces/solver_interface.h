#pragma once
#include <stdexcept>
#include <utility>

#include "Common/common.h"
#include "Constraints/bounds.h"
#include "Constraints/collision.h"
#include "Cost/cost.h"
#include "Model/integrator.h"
#include "PathSegment/path.h"

class CostFunctionManager {
public:
    CostFunctionManager(const int horizon, const std::shared_ptr<ObstaclesManager> &obstacles_manager)
        : horizon_(horizon), obstacles_manager_(obstacles_manager) {
        cost_ = std::make_shared<Cost>();
    }

    bool setCostFunction(std::shared_ptr<altro::ALTROSolver> &solver, const double desired_speed) {
        for (int i = 0; i < horizon_; ++i) {
            auto obs_list_info = obstacles_manager_->getTargetObstacleInfosAtIndex(i);
            Cost::CostFunction cost_function = cost_->CreateCostFunction(desired_speed, obs_list_info, i);
            Cost::CostJacobian cost_jacobian = cost_->CreateCostJacobian(desired_speed, obs_list_info, i);
            Cost::CostHessian cost_hessian = cost_->CreatCostHessian(desired_speed, obs_list_info, i);
            auto error_codes = solver->SetCostFunction(cost_function,
                                                       cost_jacobian,
                                                       cost_hessian,
                                                       i,
                                                       i + 1);
            if (error_codes != altro::ErrorCodes::NoError) {
                AERROR << "Failed to set cost function at horizon step " << i;
                return false;
            }
        }
        return true;
    }
    bool setFinalStateInputCost(std::shared_ptr<altro::ALTROSolver> &solver, const State &finalState, const Input &finalInput) const {
        auto error_codes = solver->SetLQRCost(NUM_STATE, NUM_INPUT, Qf.data(), Rf.data(), stateToVector(finalState).data(), inputToVector(finalInput).data(), horizon_);
        if (error_codes != altro::ErrorCodes::NoError) {
            AERROR << "Failed to set final state input cost at horizon step " << horizon_;
            return false;
        }
        return true;
    }

private:
    int horizon_;
    std::shared_ptr<Cost> cost_;
    std::shared_ptr<ObstaclesManager> obstacles_manager_;
};

template <int NDISK>
class CollisionConstraintManager {
public:
    explicit CollisionConstraintManager(const std::shared_ptr<ObstaclesManager> &obstacles_manager) : obstacles_manager_(obstacles_manager) {}

    bool setCollisionConstraints(const std::shared_ptr<altro::ALTROSolver> &solver, const int horizon) {
        for (int i = 1; i < horizon; ++i) {
            auto obs_list_info = obstacles_manager_->getTargetObstacleInfosAtIndex(i);
            auto collision_function = collision_risk_evaluator_->CreateCollisionFunction(obs_list_info);
            auto collision_jacobian = collision_risk_evaluator_->CreateCollisionJacobian(obs_list_info);
            auto error_codes = solver->SetConstraint(collision_function,
                                                     collision_jacobian,
                                                     NDISK,
                                                     altro::ConstraintType::INEQUALITY,
                                                     "Collision Constraint" + std::to_string(i),
                                                     i,
                                                     i + 1);
            if (error_codes != altro::ErrorCodes::NoError) {
                AERROR << "Failed to set collision constraint at horizon step " << i;
                return false;
            }
        }
        return true;
    }

private:
    std::shared_ptr<ObstaclesManager> obstacles_manager_;
    std::shared_ptr<CollisionRiskEvaluator<NDISK>> collision_risk_evaluator_;
};

class DynamicsManager {
public:
    explicit DynamicsManager(const std::shared_ptr<Path> &curve) {
        model_ = std::make_shared<Integrator>(curve);
    };
    bool setDynamicsFunction(std::shared_ptr<altro::ALTROSolver> &solver, int horizon) {
        auto error_codes = solver->SetExplicitDynamics(model_->dynamics_function,
                                                       model_->dynamics_jacobian,
                                                       0,
                                                       horizon);
        if (error_codes != altro::ErrorCodes::NoError) {
            AERROR << "Failed to set dynamics function";
            return false;
        }
        return true;
    }

private:
    std::shared_ptr<Integrator> model_;
};

class BoundsConstraintManager {
public:
    explicit BoundsConstraintManager() {
        bounds_ = std::make_shared<Bounds>();
    };
    bool setBoundsConstraint(std::shared_ptr<altro::ALTROSolver> &solver, int horizon) {
        if (!setLowerBoundsConstraint(solver, horizon)) {
            return false;
        }
        if (!setUpperBoundsConstraint(solver, horizon)) {
            return false;
        }
        return true;
    }

private:
    bool setLowerBoundsConstraint(std::shared_ptr<altro::ALTROSolver> &solver, int horizon) {
        for (int i = 1; i < horizon; ++i) {
            auto error_codes = solver->SetConstraint(bounds_->lower_bound_function,
                                                     bounds_->lower_bound_jacobian,
                                                     NUM_STATE + NUM_INPUT,
                                                     altro::ConstraintType::INEQUALITY,
                                                     "Lower Bound Constraint",
                                                     i,
                                                     i + 1);
            if (error_codes != altro::ErrorCodes::NoError) {
                AERROR << "Failed to set lower bound constraint at horizon step " << i;
                return false;
            }
        }
        return true;
    }
    bool setUpperBoundsConstraint(std::shared_ptr<altro::ALTROSolver> &solver, int horizon) {
        for (int i = 1; i < horizon; ++i) {
            auto error_codes = solver->SetConstraint(bounds_->upper_bound_function,
                                                     bounds_->upper_bound_jacobian,
                                                     NUM_STATE + NUM_INPUT,
                                                     altro::ConstraintType::INEQUALITY,
                                                     "Upper Bound Constraint",
                                                     i,
                                                     i + 1);
            if (error_codes != altro::ErrorCodes::NoError) {
                AERROR << "Failed to set upper bound constraint at horizon step " << i;
                return false;
            }
        }
        return true;
    }

private:
    std::shared_ptr<Bounds> bounds_;
};
