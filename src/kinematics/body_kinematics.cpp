#include "robot_model/kinematics/body_kinematics.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>

namespace robot_model {

// calc the kinematics of a specified link from base
// using pinocchio's forward kinematics algorithms to calculate the
// pose, velocity and acceleration of a body frame
std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> calcBodyKinematics(
    RobotModel& model, 
    const std::string& link_name, 
    const Eigen::VectorXd* q, 
    const Eigen::VectorXd* qp, 
    const Eigen::VectorXd* qpp)
{
    auto frame_id = model.getModel().getFrameId(link_name);
    
    if (q) {
        model.updateData(*q, qp, qpp);
    }

    const auto& position = model.getData().oMf[frame_id];
    Eigen::Vector3d translation = position.translation();
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(position.rotation());

    Eigen::VectorXd pose(6);
    pose << translation, rpy;

    auto speed = pinocchio::getFrameVelocity(model.getModel(), model.getData(), frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    Eigen::VectorXd speed_vec(6);
    speed_vec << speed.linear(), speed.angular();

    auto acceleration = pinocchio::getFrameClassicalAcceleration(model.getModel(), model.getData(), frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    Eigen::VectorXd acceleration_vec(6);
    acceleration_vec << acceleration.linear(), acceleration.angular();

    return {pose, speed_vec, acceleration_vec};
}

} // namespace robot_model
