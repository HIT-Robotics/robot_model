#ifndef ROBOT_MODEL_CONSTRAINTS_SPHERICAL_JOINT_CONSTRAINT_HPP
#define ROBOT_MODEL_CONSTRAINTS_SPHERICAL_JOINT_CONSTRAINT_HPP

#include "robot_model/constraints/fixed_constraint.hpp"

namespace robot_model {

/// Spherical (ball-and-socket) joint constraint.
///
/// Constrains only the translational error (3 DOFs).
/// Derived from FixedConstraint — all 6D math is inherited;
/// only the projection matrix is overridden to select the first 3 rows
/// (translation), leaving rotational DOFs unconstrained.
class SphericalJointConstraint : public FixedConstraint {
public:
    SphericalJointConstraint() = default;
    explicit SphericalJointConstraint(const urdf::ConstraintSharedPtr& urdf_data);

protected:
    /// Returns a (3 × 6) matrix that selects only the translational rows.
    Eigen::MatrixXd projection() const override;
};

} // namespace robot_model

#endif // ROBOT_MODEL_CONSTRAINTS_SPHERICAL_JOINT_CONSTRAINT_HPP
