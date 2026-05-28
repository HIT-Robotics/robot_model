#ifndef ROBOT_MODEL_MODEL_UTILS_HPP
#define ROBOT_MODEL_MODEL_UTILS_HPP

#include <pinocchio/multibody/model.hpp>
#include <vector>
#include <string>

namespace robot_model {

// return the list of joint IDs forming the kinematic chain from base_link_name to link_name.
inline std::vector<pinocchio::JointIndex> getJointChainToLink(
    const pinocchio::Model& model, 
    const std::string& link_name, 
    const std::string& base_link_name = "base_link") 
{
    pinocchio::FrameIndex current_frame = model.getFrameId(link_name);
    pinocchio::FrameIndex base_frame = model.getFrameId(base_link_name);
    std::vector<pinocchio::JointIndex> joint_ids;

    while (current_frame != base_frame) {
        // Joint that created this frame
        pinocchio::JointIndex parent_joint = model.frames[current_frame].parentJoint;
        
        // Reached universe but not the base link -> disconnected?
        if (parent_joint == 0) {
            break;
        }
        
        joint_ids.push_back(parent_joint);
        
        // Move to the joint's parent frame in the kinematic tree
        current_frame = model.frames[current_frame].parentFrame;
    }
    
    std::reverse(joint_ids.begin(), joint_ids.end());
    return joint_ids;
}

} // namespace robot_model

#endif // ROBOT_MODEL_MODEL_UTILS_HPP
