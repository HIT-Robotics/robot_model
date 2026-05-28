#ifndef ROBOT_MODEL_KINEMATICS_BODY_KINEMATICS_HPP
#define ROBOT_MODEL_KINEMATICS_BODY_KINEMATICS_HPP

#include "robot_model/model/robot_model.hpp"
#include <Eigen/Dense>
#include <string>
#include <tuple>

namespace robot_model {

// calculate kinematic properties of a body in the tree structure
// returns {pose, velocity, acceleration}
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> calcBodyKinematics(
    RobotModel& model, 
    const std::string& link, 
    const Eigen::VectorXd* q = nullptr, 
    const Eigen::VectorXd* qp = nullptr, 
    const Eigen::VectorXd* qpp = nullptr);

} // namespace robot_model

#endif // ROBOT_MODEL_KINEMATICS_BODY_KINEMATICS_HPP
