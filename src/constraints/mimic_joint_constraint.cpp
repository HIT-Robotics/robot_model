#include "robot_model/constraints/mimic_joint_constraint.hpp"
#include <urdf_model/joint.h>
#include <stdexcept>

namespace robot_model {

// create the mimic constraint from the URDF data
MimicJointConstraint::MimicJointConstraint(const urdf::JointSharedPtr& urdf_joint)
    : urdf_joint_(urdf_joint)
{
    if (!urdf_joint->mimic) {
        throw std::runtime_error("Joint '" + urdf_joint->name + "' is not a mimic joint");
    }
    multiplier_ = urdf_joint->mimic->multiplier;
    offset_ = urdf_joint->mimic->offset;
}

// initialize the mimic constraint by saving the ids of the parent and child joints
pinocchio::Model& MimicJointConstraint::initialize(pinocchio::Model& model) {
    if (!model.existJointName(urdf_joint_->name)) {
        throw std::runtime_error("Joint '" + urdf_joint_->name + "' not found in pinocchio model");
    }
    if (!model.existJointName(urdf_joint_->mimic->joint_name)) {
        throw std::runtime_error("Parent mimic joint '" + urdf_joint_->mimic->joint_name + "' not found in pinocchio model");
    }

    child_id_ = model.getJointId(urdf_joint_->name);
    parent_id_ = model.getJointId(urdf_joint_->mimic->joint_name);

    return model;
}

// return the dimension of the constraint
int MimicJointConstraint::dim() const {
    return 1;
}

// calculate the constraint equation
Eigen::VectorXd MimicJointConstraint::h(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q) const {
    const auto& parent_joint = model.joints[parent_id_];
    const auto& child_joint = model.joints[child_id_];
    
    double parent_q = q(parent_joint.idx_v());
    double child_q = q(child_joint.idx_v());
    
    Eigen::VectorXd res(1);
    res(0) = parent_q * multiplier_ - child_q + offset_;
    return res;
}

// calculate the jacobian of the constraint
Eigen::MatrixXd MimicJointConstraint::j(const pinocchio::Model& model, pinocchio::Data& data) const {
    const auto& parent_joint = model.joints[parent_id_];
    const auto& child_joint = model.joints[child_id_];
    
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(1, model.nv);
    res(0, child_joint.idx_v()) = -1.0;
    res(0, parent_joint.idx_v()) = multiplier_;
    return res;
}

// return the derivative of the jacobian of the constraint
Eigen::MatrixXd MimicJointConstraint::jp(const pinocchio::Model& model, pinocchio::Data& data) const {
    return Eigen::MatrixXd::Zero(1, model.nv);
}

} // namespace robot_model
