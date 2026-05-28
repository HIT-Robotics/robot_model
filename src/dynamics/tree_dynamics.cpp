#include "robot_model/dynamics/tree_dynamics.hpp"
#include <pinocchio/algorithm/rnea.hpp>

namespace robot_model {

// compute the inverse dynamics of a tree structure
// using pinocchio's recursive newton-euler algorithm
Eigen::VectorXd computeInverseDynamicsTreeStructure(
    RobotModel& model, 
    const Eigen::VectorXd& q, 
    const Eigen::VectorXd& qp, 
    const Eigen::VectorXd& qpp, 
    bool update_data)
{
    if (update_data) {
        model.updateData(q, &qp, &qpp);
    }
    return pinocchio::rnea(model.getModel(), model.getData(), model.q_state(), model.qp(), model.qpp());
}

} // namespace robot_model
