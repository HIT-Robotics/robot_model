#ifndef ROBOT_MODEL_CONSTRAINTS_MIMIC_JOINT_CONSTRAINT_HPP
#define ROBOT_MODEL_CONSTRAINTS_MIMIC_JOINT_CONSTRAINT_HPP

#include "robot_model/constraints/constraint.hpp"
#include <urdf_model/joint.h>

namespace robot_model {

// mimic joint constraint to handle mimic joints as a constraint and solve them with the levenberg-marquardt algorithm
class MimicJointConstraint : public Constraint {
public:
    MimicJointConstraint() = default;
    explicit MimicJointConstraint(const urdf::JointSharedPtr& urdf_joint);

    pinocchio::Model& initialize(pinocchio::Model& model) override;
    int dim() const override;
    virtual Eigen::VectorXd h(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q) const override;
    virtual Eigen::MatrixXd j(const pinocchio::Model& model, pinocchio::Data& data) const override;
    virtual Eigen::MatrixXd jp(const pinocchio::Model& model, pinocchio::Data& data) const override;
    
    void setUrdfJoint(const urdf::JointSharedPtr& urdf_joint){
        urdf_joint_ = urdf_joint;
    }
    
protected:
    urdf::JointSharedPtr urdf_joint_;
    pinocchio::JointIndex parent_id_;
    pinocchio::JointIndex child_id_;
    double multiplier_;
    double offset_;
};

} // namespace robot_model

#endif // ROBOT_MODEL_CONSTRAINTS_MIMIC_JOINT_CONSTRAINT_HPP
