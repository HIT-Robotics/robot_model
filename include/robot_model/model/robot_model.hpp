#ifndef ROBOT_MODEL_MODEL_ROBOT_MODEL_HPP
#define ROBOT_MODEL_MODEL_ROBOT_MODEL_HPP

#include "robot_model/constraints/constraint_manager.hpp"
#include "robot_model/model/limits.hpp"
#include "robot_model/model/joints.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <urdf_model/model.h>
#include <memory>
#include <string>
#include <vector>

namespace robot_model {

// class to manage the robot model
// it combines all the other classes to provide a complete interface for the robot model
// this is the main class that will be used by the user
class RobotModel {
public:
    RobotModel(const std::string& path);

    void updateData(const Eigen::VectorXd& q, 
                    const Eigen::VectorXd* qp = nullptr, 
                    const Eigen::VectorXd* qpp = nullptr, 
                    bool calc_jacobian = false, 
                    bool calc_jacobian_time_derivative = false);

    void calcJacobian();
    void calcJacobianTimeVariation(const Eigen::VectorXd* qp = nullptr);

    Eigen::VectorXd q_motion() const { return q_motion_; }
    Eigen::VectorXd q_state() const { return q_state_; }
    Eigen::VectorXd qp() const { return qp_; }
    Eigen::VectorXd qpp() const { return qpp_; }

    Eigen::VectorXd motionToStateJoint(const Eigen::VectorXd& q) const;
    Eigen::VectorXd stateToMotionJoint(const Eigen::VectorXd& q) const;

    Eigen::VectorXd h(const Eigen::VectorXd* q = nullptr, const Eigen::VectorXd* qp = nullptr, const Eigen::VectorXd* qpp = nullptr);
    Eigen::MatrixXd j(const Eigen::VectorXd* q = nullptr, const Eigen::VectorXd* qp = nullptr, const Eigen::VectorXd* qpp = nullptr);
    Eigen::MatrixXd j_d(const Eigen::VectorXd* q = nullptr, const Eigen::VectorXd* qp = nullptr, const Eigen::VectorXd* qpp = nullptr);

    void addTooling(const pinocchio::Inertia& inertia, const pinocchio::SE3& center_of_gravity, const std::string& joint = "");

    int degrees_of_freedom() const { return model_.nv - constraints_->dim(); }

    Eigen::VectorXi checkJointLimits(const Eigen::VectorXd* q = nullptr, const Eigen::VectorXd* qp = nullptr, const Eigen::VectorXd* qpp = nullptr);
    bool isStateValid(const Eigen::VectorXd* q = nullptr, const Eigen::VectorXd* qp = nullptr, const Eigen::VectorXd* qpp = nullptr);

    Eigen::VectorXd normalizeRotationalJoints(const Eigen::VectorXd& q) const;
    Eigen::VectorXd neutralStateConfiguration() const;
    Eigen::VectorXd neutralMotionConfiguration() const;
    Eigen::VectorXd randomStateConfiguration() const;
    Eigen::VectorXd randomMotionConfiguration() const;
    Eigen::VectorXd createRandom(const Eigen::VectorXi& random_mask, const Eigen::VectorXd* q = nullptr);

    pinocchio::Model& getModel() { return model_; }
    const pinocchio::Model& getModel() const { return model_; }
    pinocchio::Data& getData() { return *data_; }
    const pinocchio::Data& getData() const { return *data_; }
    ConstraintManager& getConstraints() { return *constraints_; }
    const ConstraintManager& getConstraints() const { return *constraints_; }
    LimitManager& getLimits() { return *limits_; }
    const LimitManager& getLimits() const { return *limits_; }
    JointManager& getJoints() { return *joints_; }
    const JointManager& getJoints() const { return *joints_; }

private:
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;

    std::unique_ptr<ConstraintManager> constraints_;
    std::unique_ptr<LimitManager> limits_;
    std::unique_ptr<JointManager> joints_;
    std::shared_ptr<urdf::ModelInterface> urdf_model_;
    
    bool jacobian_valid_;
    bool jacobian_time_valid_;

    Eigen::VectorXd q_motion_;
    Eigen::VectorXd q_state_;
    Eigen::VectorXd qp_;
    Eigen::VectorXd qpp_;

    std::vector<Transmission> parseTransmissions(const std::string& path);
};

} // namespace robot_model

#endif // ROBOT_MODEL_MODEL_ROBOT_MODEL_HPP
