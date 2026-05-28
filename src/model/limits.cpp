#include "robot_model/model/limits.hpp"
#include <iostream>

namespace robot_model {

// constructor
LimitManager::LimitManager(const urdf::ModelInterface& urdf_model) {
    // iterate over all joints in the URDF model
    for (const auto& pair : urdf_model.joints_) {
        const auto& urdf_joint = pair.second;
        // check if the joint has limits
        if (urdf_joint->limits) {
            JointLimit limit;
            limit.name = urdf_joint->name;
            // check if the joint is continuous
            if (urdf_joint->type != urdf::Joint::CONTINUOUS) {
                // set the lower and upper limits
                limit.lower = urdf_joint->limits->lower;
                limit.upper = urdf_joint->limits->upper;
            }
            else {
                limit.lower = -std::numeric_limits<double>::infinity();
                limit.upper = std::numeric_limits<double>::infinity();
            }
            limit.velocity = urdf_joint->limits->velocity;
            limit.effort = urdf_joint->limits->effort;
            joint_limits_.push_back(limit);
        }
    }
}

// Checks the joint limits for the given joint configuration and return a mask with violations
Eigen::VectorXi LimitManager::checkJointLimits(const pinocchio::Model& model, const Eigen::VectorXd& q) {
    Eigen::VectorXi mask = Eigen::VectorXi::Zero(model.nv);
    for (const auto& limit : joint_limits_) {
        if (!model.existJointName(limit.name)) continue;
        auto joint_id = model.getJointId(limit.name);
        const auto& joint = model.joints[joint_id];
        
        if (joint.nv() == 1) {
            double val = q(joint.idx_v());
            if (val < limit.lower || val > limit.upper) {
                mask(joint.idx_v()) = 1;
            }
        }
    }
    return mask;
}

// checks if the state is valid, returning false if any joint is out of limits
bool LimitManager::stateValid(const pinocchio::Model& model, const pinocchio::Data& data, const Eigen::VectorXd& q) {
    Eigen::VectorXi joint_mask = checkJointLimits(model, q);
    if (joint_mask.any()) return false;
    return true;
}

} // namespace robot_model
