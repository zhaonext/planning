#include <iostream>

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/augmented/SlacknessSquaredHingePenalty.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/ILQR.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/RolloutSettings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>


#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <memory>

#include "dynamic.h"


std::unique_ptr<ocs2::augmented::AugmentedPenaltyBase> getPenalty() {
    using penalty_type = ocs2::augmented::SlacknessSquaredHingePenalty;
    penalty_type::Config boundsConfig;
    return penalty_type::create(boundsConfig);
}

int main() {

    auto ddpSettings = ocs2::ddp::Settings();
    ddpSettings.algorithm_ = ocs2::ddp::Algorithm::ILQR;
    ddpSettings.maxNumIterations_ = 200;
    ddpSettings.timeStep_ = 0.1;
    ddpSettings.displayShortSummary_ = true;

    // auto mpcSettings = ocs2::mpc::Settings();
    auto problem = ocs2::OptimalControlProblem();

    std::string libraryFolder = boost::filesystem::current_path().string() + "/auto_generated";
    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    problem.dynamicsPtr = std::make_unique<VehicleDynamics>(libraryFolder, false);

    problem.costPtr->add("cost", std::make_unique<VehicleKinematicsCost>(libraryFolder));

    std::unordered_map<std::string, size_t> a;
    a["xx"] = 1;
    std::unordered_map<std::string, size_t> b(a);

    problem.inequalityLagrangianPtr->add("constraint", ocs2::create(std::make_unique<VehicleKinematicsConstraint>(libraryFolder), getPenalty()));

    problem.finalCostPtr->add("finalCost", std::make_unique<finalVehicleKinematicsCost>(libraryFolder));

    auto rolloutSettings = ocs2::rollout::Settings();
    rolloutSettings.timeStep = 0.1;
    std::unique_ptr<ocs2::RolloutBase> rolloutPtr = std::make_unique<ocs2::TimeTriggeredRollout>(*problem.dynamicsPtr, rolloutSettings);
    std::unique_ptr<ocs2::Initializer> initializerPtr = std::make_unique<ocs2::DefaultInitializer>(2);

    auto ddpPtr = new ocs2::ILQR(ddpSettings, *rolloutPtr, problem, *initializerPtr);

    ocs2::TargetTrajectories initTargetTrajectories;
    initTargetTrajectories.timeTrajectory.push_back(0.0);
    initTargetTrajectories.stateTrajectory.emplace_back(Eigen::VectorXd::Zero(3));
    initTargetTrajectories.inputTrajectory.emplace_back(Eigen::VectorXd::Zero(2));
    ddpPtr->getReferenceManager().setTargetTrajectories(initTargetTrajectories);

    Eigen::VectorXd initialState = Eigen::VectorXd::Zero(3);
    ddpPtr->run(0, initialState, 10);

    auto finalState = ddpPtr->primalSolution(10).stateTrajectory_.back();


    std::cout
            << "x = " << finalState(0) << ", y = " << finalState(1) << ", z = " << finalState(2) << std::endl;
    return 0;
}
