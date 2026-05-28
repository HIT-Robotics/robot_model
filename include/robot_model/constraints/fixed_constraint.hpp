#ifndef ROBOT_MODEL_CONSTRAINTS_FIXED_CONSTRAINT_HPP
#define ROBOT_MODEL_CONSTRAINTS_FIXED_CONSTRAINT_HPP

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <urdf_model/constraint.h>
#include "robot_model/constraints/constraint.hpp"

namespace robot_model {

/// Full 6-DOF rigid constraint between two frames.
///
/// Implements the complete translational (3D) and rotational (3D) error,
/// Jacobian, and Jacobian time-derivative.  All quantities are expressed in
/// the LOCAL_WORLD_ALIGNED reference frame.
///
/// Derived classes can reduce the constraint to a subset of DOFs by overriding
/// projection().  The projection matrix P has shape (k × 6) where k ≤ 6:
///   h  = P * h_full   (k-vector)
///   J  = P * J_full   (k × nv)
///   Jp = P * Jp_full  (k × nv)
///
/// dim() is automatically inferred as P.rows().
///
/// Corresponds to urdf::Constraint::LINK type by default; subclasses may be
/// registered for other types in the ConstraintManager.
class FixedConstraint : public Constraint {
public:
    FixedConstraint() = default;
    explicit FixedConstraint(const urdf::ConstraintSharedPtr& urdf_data): urdf_data_(urdf_data){}
    // setter for the urdf data
    void setUrdfData(const urdf::ConstraintSharedPtr& urdf_data){
        urdf_data_ = urdf_data;
    }
    /// Registers parent and child OP_FRAMEs into the pinocchio model and initialize other data.
    pinocchio::Model& initialize(pinocchio::Model& model) override;
    pinocchio::Model& initialize(pinocchio::Model& model, urdf::ConstraintSharedPtr urdf_data);

    /// Returns the number of active constraint rows: projection().rows().
    int dim() const override;

    Eigen::VectorXd h(const pinocchio::Model& model, pinocchio::Data& data,
                      const Eigen::VectorXd& q) const override;

    Eigen::MatrixXd j(const pinocchio::Model& model,
                      pinocchio::Data& data) const override;

    Eigen::MatrixXd jp(const pinocchio::Model& model,
                       pinocchio::Data& data) const override;

protected:
    // -----------------------------------------------------------------------
    // Full 6D (unprojected) computations — available to all subclasses
    // -----------------------------------------------------------------------

    /// 6-vector: [ p_p - p_c ;  0.5 * vee(R_rel - R_rel^T) ]
    Eigen::VectorXd h_full(const pinocchio::Model& model, pinocchio::Data& data) const;

    /// 6×nv matrix: full constraint Jacobian
    Eigen::MatrixXd j_full(const pinocchio::Model& model, pinocchio::Data& data) const;

    /// 6×nv matrix: time derivative of the full constraint Jacobian
    Eigen::MatrixXd jp_full(const pinocchio::Model& model, pinocchio::Data& data) const;

    // -----------------------------------------------------------------------
    // Projection — override in derived classes to select active DOFs
    // -----------------------------------------------------------------------

    /// Returns the (k × 6) projection matrix applied to all full quantities.
    /// Default: identity (all 6 DOFs active).
    virtual Eigen::MatrixXd projection() const;

    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------
    pinocchio::FrameIndex parent_id_{0};
    pinocchio::FrameIndex child_id_{0};
    urdf::ConstraintSharedPtr urdf_data_=nullptr;
};

} // namespace robot_model

#endif // ROBOT_MODEL_CONSTRAINTS_FIXED_CONSTRAINT_HPP
