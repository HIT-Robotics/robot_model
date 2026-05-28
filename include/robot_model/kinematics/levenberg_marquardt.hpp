#ifndef ROBOT_MODEL_KINEMATICS_LEVENBERG_MARQUARDT_HPP
#define ROBOT_MODEL_KINEMATICS_LEVENBERG_MARQUARDT_HPP

#include "robot_model/model/robot_model.hpp"
#include <Eigen/Dense>
#include <variant>

namespace robot_model {

    // enumeration to describe the desired robot configuration regarding the passive angles of a delta robot. Upper meas that the endeffector lies above the first passive joints. Default value is LOWER
enum class RobotConfiguration {
    UPPER,
    LOWER
};

// Using Levenberg-Marquardt Algorithm to solve the kinematics based on the constraints
std::pair<Eigen::VectorXd, bool> levenbergMarquardt(
    RobotModel& robot_model, 
    std::variant<Eigen::VectorXi, CoordinateType> mask_independent, 
    const Eigen::VectorXd* q0 = nullptr, 
    double tol = 1e-6, 
    int max_iterations = 50, 
    int max_step_line_search = 10);

// calculate a valid motion state based on the Levenberg Marquardt algorithm
std::pair<Eigen::VectorXd, bool> calcValidMotionState(
    RobotModel& model, 
    RobotConfiguration configuration = RobotConfiguration::LOWER, 
    double tol = 1e-10, 
    int num_iterations = 100, 
    int max_step_line_search = 10);

} // namespace robot_model

#endif // ROBOT_MODEL_KINEMATICS_LEVENBERG_MARQUARDT_HPP
