#include "robot_model/constraints/spherical_joint_constraint.hpp"

namespace robot_model {

// Constructor to initialize with a constraint object
SphericalJointConstraint::SphericalJointConstraint(const urdf::ConstraintSharedPtr& urdf_data)
    : FixedConstraint(urdf_data)
{}

// returns a (3x6) matrix that selects only the translational rows
Eigen::MatrixXd SphericalJointConstraint::projection() const {
    return Eigen::MatrixXd::Identity(6, 6).topRows<3>();
}

} // namespace robot_model
