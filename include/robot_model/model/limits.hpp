#ifndef ROBOT_MODEL_MODEL_LIMITS_HPP
#define ROBOT_MODEL_MODEL_LIMITS_HPP

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <urdf_model/model.h>
#include <vector>
#include <string>

namespace robot_model {

// struct to store joint limits
struct JointLimit {
    std::string name;
    double lower;
    double upper;
    double velocity;
    double effort;
};

// class to manage the joint limits
class LimitManager {
public:
    LimitManager(const urdf::ModelInterface& urdf_model);

    Eigen::VectorXi checkJointLimits(const pinocchio::Model& model, const Eigen::VectorXd& q);
    bool stateValid(const pinocchio::Model& model, const pinocchio::Data& data, const Eigen::VectorXd& q);

private:
    std::vector<JointLimit> joint_limits_;
};

} // namespace robot_model

#endif // ROBOT_MODEL_MODEL_LIMITS_HPP
