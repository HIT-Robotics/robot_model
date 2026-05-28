#ifndef ROBOT_MODEL_CONSTRAINTS_CONSTRAINT_HPP
#define ROBOT_MODEL_CONSTRAINTS_CONSTRAINT_HPP

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <urdf_model/model.h>
#include <memory>

namespace robot_model {

// interface for all constraints
class Constraint {
public:
    virtual ~Constraint() = default;

    virtual pinocchio::Model& initialize(pinocchio::Model& model) = 0;
    virtual int dim() const = 0;
    virtual Eigen::VectorXd h(const pinocchio::Model& model, pinocchio::Data& data, const Eigen::VectorXd& q) const = 0;
    virtual Eigen::MatrixXd j(const pinocchio::Model& model, pinocchio::Data& data) const = 0;
    virtual Eigen::MatrixXd jp(const pinocchio::Model& model, pinocchio::Data& data) const = 0;
};

} // namespace robot_model

#endif // ROBOT_MODEL_CONSTRAINTS_CONSTRAINT_HPP
