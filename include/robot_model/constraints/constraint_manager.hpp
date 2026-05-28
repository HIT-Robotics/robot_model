#ifndef ROBOT_MODEL_CONSTRAINTS_CONSTRAINT_MANAGER_HPP
#define ROBOT_MODEL_CONSTRAINTS_CONSTRAINT_MANAGER_HPP

#include "robot_model/constraints/constraint.hpp"
#include <urdf_model/model.h>
#include <vector>
#include <memory>

namespace robot_model {

// constraint manager to create constraints based on an extended URDF and calculate the contraint error, jacobian and jacobian derivative
class ConstraintManager : public Constraint {
public:
    ConstraintManager() = default;
    ConstraintManager(const ConstraintManager&) = delete;
    ConstraintManager& operator=(const ConstraintManager&) = delete;

    pinocchio::Model& initialize(pinocchio::Model& model, const urdf::ModelInterface& urdf_model);

    pinocchio::Model& initialize(pinocchio::Model& model) override { return model; }

    int dim() const override;
    virtual Eigen::VectorXd h(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q) const override;
    virtual Eigen::MatrixXd j(const pinocchio::Model& model, pinocchio::Data& data) const override;
    virtual Eigen::MatrixXd jp(const pinocchio::Model& model, pinocchio::Data& data) const override;

private:
    std::vector<std::unique_ptr<Constraint>> constraints_;
};

} // namespace robot_model

#endif // ROBOT_MODEL_CONSTRAINTS_CONSTRAINT_MANAGER_HPP
