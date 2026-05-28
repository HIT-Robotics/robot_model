#include "robot_model/constraints/revolute_joint_constraint.hpp"
#include <cmath>

namespace robot_model {

// Constructor to initialize with a constraint object 
RevoluteJointConstraint::RevoluteJointConstraint(const urdf::ConstraintSharedPtr& urdf_data)
    : FixedConstraint(urdf_data)
{}

// returns a (5x6) projection matrix that keeps translation fixed and constrains rotation around the parent axis
// as helper to calculate the projection two helper axis are selected, such that they are orthogonal to the parent axis
Eigen::MatrixXd RevoluteJointConstraint::projection() const {
    // Parent axis defined in the parent frame
    Eigen::Vector3d a(urdf_data_->parent_axis.x, urdf_data_->parent_axis.y, urdf_data_->parent_axis.z);
    a.normalize();

    // Find a helper vector that is not collinear with a
    Eigen::Vector3d helper = (std::abs(a.x()) > 0.9) ? Eigen::Vector3d(0.0, 1.0, 0.0) : Eigen::Vector3d(1.0, 0.0, 0.0);

    // Compute the orthonormal basis vectors e1, e2 orthogonal to axis 'a'
    Eigen::Vector3d e1 = a.cross(helper);
    e1.normalize();
    Eigen::Vector3d e2 = a.cross(e1);
    e2.normalize(); // e2 is already unit length, but good for precision

    // Create the projection matrix P of size (5 x 6)
    // Layout of the 6D constraint vector: [ translation(3) | rotation(3) ]
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, 6);

    // 3 translations are fully constrained
    P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    // 2 rotations orthogonal to the axis are constrained
    P.block<1, 3>(3, 3) = e1.transpose();
    P.block<1, 3>(4, 3) = e2.transpose();

    return P;
}

} // namespace robot_model
