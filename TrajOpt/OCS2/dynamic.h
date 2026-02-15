#pragma once

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_core/cost/StateCostCppAd.h>
#include <ocs2_core/cost/StateInputCostCppAd.h>
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

class VehicleDynamics : public ocs2::SystemDynamicsBaseAD {
public:
    VehicleDynamics(const std::string &libraryFolder, bool verbose) {
        initialize(3, 2, "vehicle_dynamics", libraryFolder, true, verbose);
    }

    ~VehicleDynamics() override = default;

    VehicleDynamics(const VehicleDynamics &rhs) = default;

    VehicleDynamics *clone() const override { return new VehicleDynamics(*this); }

    ocs2::ad_vector_t systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state, const ocs2::ad_vector_t &input,
                                    const ocs2::ad_vector_t &parameters) const override {
        const ocs2::ad_scalar_t &theta = state(2);
        const ocs2::ad_scalar_t cos_theta = cos(theta);
        const ocs2::ad_scalar_t sin_theta = sin(theta);

        const ocs2::ad_scalar_t &velocity = input(0);
        const ocs2::ad_scalar_t &kappa = input(1);

        ocs2::ad_vector_t stateDerivative(3);
        stateDerivative << velocity * cos_theta, velocity * sin_theta, velocity * kappa;
        return stateDerivative;
    }
};

class VehicleKinematicsCost : public ocs2::StateInputCostCppAd {
public:
    VehicleKinematicsCost(const std::string &libraryPath = "/tmp/ocs2") {
        initialize(3, 2, 0, "vehicle_kinematics_cost", libraryPath, true, false);
    }

    VehicleKinematicsCost *clone() const override { return new VehicleKinematicsCost(*this); }

    ocs2::ad_scalar_t costFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state, const ocs2::ad_vector_t &input,
                                   const ocs2::ad_vector_t &parameters) const override {
        const ocs2::ad_scalar_t &velocity = input(0);
        const ocs2::ad_scalar_t &kappa = input(1);

        return velocity * velocity + 10 * kappa * kappa;
    }
};

class finalVehicleKinematicsCost : public ocs2::StateCostCppAd {
public:
    finalVehicleKinematicsCost(const std::string &libraryPath = "/tmp/ocs2") {
        initialize(3, 0, "final_vehicle_kinematics_cost", libraryPath, true, false);
    }

    finalVehicleKinematicsCost *clone() const override { return new finalVehicleKinematicsCost(*this); }

    ocs2::ad_scalar_t costFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state,
                                   const ocs2::ad_vector_t &parameters) const override {
        const ocs2::ad_scalar_t &x = state(0);
        const ocs2::ad_scalar_t &y = state(1);
        const ocs2::ad_scalar_t &theta = state(2);
        const ocs2::ad_scalar_t cos_theta = cos(theta);
        const ocs2::ad_scalar_t sin_theta = sin(theta);

        auto target_x = 10., target_y = 1., target_theta = 0.;
        return 10 * (x - target_x) * (x - target_x) + 10 * (y - target_y) * (y - target_y) +
               100 * (cos_theta - cos(target_theta)) * (cos_theta - cos(target_theta)) +
               100 * (sin_theta - sin(target_theta)) * (sin_theta - sin(target_theta));
    }
};

class VehicleKinematicsConstraint : public ocs2::StateInputConstraintCppAd {
public:
    VehicleKinematicsConstraint(const std::string &libraryPath = "/tmp/ocs2") : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Quadratic) {
        initialize(3, 2, 0, "vehicle_kinematics_constraint", libraryPath, true, false);
    }

    VehicleKinematicsConstraint *clone() const override { return new VehicleKinematicsConstraint(*this); }

    size_t getNumConstraints(ocs2::scalar_t time) const override { return 2; }

    ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state, const ocs2::ad_vector_t &input,
                                         const ocs2::ad_vector_t &parameters) const override {
        ocs2::ad_vector_t constraint(2);

        const ocs2::ad_scalar_t &velocity = input(0);
        const ocs2::ad_scalar_t &kappa = input(1);

        constraint(0) = 0.2 - kappa;
        constraint(1) = kappa + 0.2;

        return constraint;
    }
};