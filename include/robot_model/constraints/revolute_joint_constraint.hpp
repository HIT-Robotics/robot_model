#ifndef ROBOT_MODEL_CONSTRAINTS_REVOLUTE_JOINT_CONSTRAINT_HPP
#define ROBOT_MODEL_CONSTRAINTS_REVOLUTE_JOINT_CONSTRAINT_HPP

#include "robot_model/constraints/fixed_constraint.hpp"

namespace robot_model {

/// Revolute joint constraint.
///
/// Constrains 5 DOFs:
/// - 3 translation DOFs are fully constrained (fixed).
/// - 2 rotational DOFs orthogonal to the joint axis are constrained (fixed),
///   allowing rotation only around the joint axis.
class RevoluteJointConstraint : public FixedConstraint {
public:
    RevoluteJointConstraint() = default;
    explicit RevoluteJointConstraint(const urdf::ConstraintSharedPtr& urdf_data);

protected:
    /// Returns a (5 × 6) projection matrix that keeps translation fixed
    /// and constrains rotation around the parent axis.
    Eigen::MatrixXd projection() const override;
};

} // namespace robot_model

#endif // ROBOT_MODEL_CONSTRAINTS_REVOLUTE_JOINT_CONSTRAINT_HPP
