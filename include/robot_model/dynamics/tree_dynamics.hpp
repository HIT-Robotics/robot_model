#ifndef ROBOT_MODEL_DYNAMICS_TREE_DYNAMICS_HPP
#define ROBOT_MODEL_DYNAMICS_TREE_DYNAMICS_HPP

#include "robot_model/model/robot_model.hpp"
#include <Eigen/Dense>

namespace robot_model {

// calculate the inverse dynamics of the tree structure (ignoring closed loops)
Eigen::VectorXd computeInverseDynamicsTreeStructure(
    RobotModel& model, 
    const Eigen::VectorXd& q, 
    const Eigen::VectorXd& qp, 
    const Eigen::VectorXd& qpp, 
    bool update_data = true);

} // namespace robot_model

#endif // ROBOT_MODEL_DYNAMICS_TREE_DYNAMICS_HPP
