#include "robot_model/constraints/constraint_manager.hpp"
#include "robot_model/constraints/spherical_joint_constraint.hpp"
#include "robot_model/constraints/fixed_constraint.hpp"
#include "robot_model/constraints/revolute_joint_constraint.hpp"
#include "robot_model/constraints/mimic_joint_constraint.hpp"
#include <urdf_model/constraint.h>
#include <urdf_model/joint.h>
#include <iostream>

namespace robot_model {

// add constraints to the pinocchio model and intialize the manager
pinocchio::Model& ConstraintManager::initialize(pinocchio::Model& model, const urdf::ModelInterface& urdf_model) {
    constraints_.clear();

    // add constraints from URDF (extended)
    for (const auto& pair : urdf_model.constraints_) {
        const auto& urdf_constraint = pair.second;
        if (urdf_constraint->type == urdf::Constraint::SPHERICAL) {
            auto constraint = std::make_unique<SphericalJointConstraint>(urdf_constraint);
            model = constraint->initialize(model);
            constraints_.push_back(std::move(constraint));
        } else if (urdf_constraint->type == urdf::Constraint::REVOLUTE) {
            auto constraint = std::make_unique<RevoluteJointConstraint>(urdf_constraint);
            model = constraint->initialize(model);
            constraints_.push_back(std::move(constraint));
        } else if (urdf_constraint->type == urdf::Constraint::LINK) {
            auto constraint = std::make_unique<FixedConstraint>(urdf_constraint);
            model = constraint->initialize(model);
            constraints_.push_back(std::move(constraint));
        } else {
            std::cerr << "Warning: Unsupported constraint type for '" << urdf_constraint->name << "'" << std::endl;
        }
    }

    // Add mimic joint constraints
    for (const auto& pair : urdf_model.joints_) {
        const auto& urdf_joint = pair.second;
        if (urdf_joint->mimic) {
            auto constraint = std::make_unique<MimicJointConstraint>(urdf_joint);
            model = constraint->initialize(model);
            constraints_.push_back(std::move(constraint));
        }
    }
    // return the updated model
    return model;
}

// return the dimension of the constraints
int ConstraintManager::dim() const {
    int total_dim = 0;
    for (const auto& c : constraints_) {
        total_dim += c->dim();
    }
    return total_dim;
}

// return the constraint values
Eigen::VectorXd ConstraintManager::h(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q) const {
    if (constraints_.empty()) return Eigen::VectorXd::Zero(0);

    Eigen::VectorXd res(dim());
    int offset = 0;
    for (const auto& c : constraints_) {
        int d = c->dim();
        if (d > 0) {
            res.segment(offset, d) = c->h(model, data, q);
            offset += d;
        }
    }
    return res;
}

// return the jacobian of the constraints
Eigen::MatrixXd ConstraintManager::j(const pinocchio::Model& model, pinocchio::Data& data) const {
    if (constraints_.empty()) return Eigen::MatrixXd::Zero(0, model.nv);

    Eigen::MatrixXd res(dim(), model.nv);
    int offset = 0;
    for (const auto& c : constraints_) {
        int d = c->dim();
        if (d > 0) {
            res.block(offset, 0, d, model.nv) = c->j(model, data);
            offset += d;
        }
    }
    return res;
}

// return the derivative of the jacobian of the constraints
Eigen::MatrixXd ConstraintManager::jp(const pinocchio::Model& model, pinocchio::Data& data) const {
    if (constraints_.empty()) return Eigen::MatrixXd::Zero(0, model.nv);

    Eigen::MatrixXd res(dim(), model.nv);
    int offset = 0;
    for (const auto& c : constraints_) {
        int d = c->dim();
        if (d > 0) {
            res.block(offset, 0, d, model.nv) = c->jp(model, data);
            offset += d;
        }
    }
    return res;
}

} // namespace robot_model
