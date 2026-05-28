#include "robot_model/model/joints.hpp"
#include "robot_model/model/utils.hpp"
#include <stdexcept>
#include <unordered_set>

namespace robot_model {

// set of joint models that are rotational
static const std::unordered_set<std::string> ROTATIONAL_JOINTS = {
    "JointModelRX", "JointModelRY", "JointModelRZ",
    "JointModelRUBX", "JointModelRUBY", "JointModelRUBZ"
};

// set of joint models that are prismatic
static const std::unordered_set<std::string> PRISMATIC_JOINTS = {
    "JointModelPrismaticUnaligned", "JointModelPX", "JointModelPY", "JointModelPZ"
};

// Converts a binary mask vector to a filter matrix.
Eigen::MatrixXd maskToFilterMatrix(const Eigen::VectorXi& mask) {
    int count = mask.sum();
    Eigen::MatrixXd filter = Eigen::MatrixXd::Zero(count, mask.size());
    int row = 0;
    for (int i = 0; i < mask.size(); ++i) {
        if (mask(i)) {
            filter(row, i) = 1.0;
            row++;
        }
    }
    return filter;
}

// Constructor
JointManager::JointManager(const pinocchio::Model& model, 
                           const urdf::ModelInterface& urdf_model,
                           const std::vector<Transmission>& transmissions,
                           const std::string& ee_link)
    : nv_(model.nv)
{
    // Calculate masks for active and passive joints, and identify rotational joints
    calcMaskEeJoints(model, ee_link);
    calcMaskRotationalJoints(model);
    calcMaskActiveJoints(transmissions, model);

    // Read gear ratio, efficiency, inertia, friction, and damping
    readGearRatio(transmissions, model);
    readGearEfficiency(transmissions, model);
    readGearInertia(transmissions, model);
    readJointFriction(urdf_model, model);
    readJointDamping(urdf_model, model);

    // Store joint information (index and size) for all joints except the fixed base joint
    for (size_t i = 1; i < model.joints.size(); ++i) {
        joint_info_[model.names[i]] = {model.joints[i].idx_v(), model.joints[i].nv()};
    }
}

// Returns the mask for the specified coordinate type
Eigen::VectorXi JointManager::getMask(CoordinateType coord) const {
    switch (coord) {
        case CoordinateType::ROBOT: return mask_robot_joints;
        case CoordinateType::EE: return mask_ee_joints;
        case CoordinateType::ACTIVE: return mask_active_joints;
        case CoordinateType::PASSIVE: return mask_passive_joints;
        case CoordinateType::ROTATIONAL: return mask_rotational_joints;
        default: throw std::invalid_argument("Unsupported coordinate type");
    }
}

// Returns a mask constructed from a list of joint names
Eigen::VectorXi JointManager::getMask(const std::vector<std::string>& joint_names) const {
    // allocate mask vector with zeros
    Eigen::VectorXi mask = Eigen::VectorXi::Zero(nv_);
    // set the mask to one for the given joint names
    for (const auto& name : joint_names) {
        auto it = joint_info_.find(name);
        if (it == joint_info_.end()) {
            throw std::invalid_argument("Joint name '" + name + "' not found in JointManager");
        }
        for (int i = 0; i < it->second.second; ++i) {
            mask(it->second.first + i) = 1;
        }
    }
    return mask;
}

// Returns filter matrix for joints connecting the ee with the base
Eigen::MatrixXd JointManager::getFilterMatrixEeJoints() const {
    return maskToFilterMatrix(mask_ee_joints);
}

// Returns filter matrix for all joints not connecting the ee with the base
Eigen::MatrixXd JointManager::getFilterMatrixRobotJoints() const {
    return maskToFilterMatrix(mask_robot_joints);
}

// Returns filter matrix for all active joints
Eigen::MatrixXd JointManager::getFilterMatrixActiveJoints() const {
    return maskToFilterMatrix(mask_active_joints);
}

// Returns the filter matrix for all passive joints
Eigen::MatrixXd JointManager::getFilterMatrixPassiveJoints() const {
    return maskToFilterMatrix(mask_passive_joints);
}

// Returns the filter matrix for all rotational joints
Eigen::MatrixXd JointManager::getFilterMatrixRotationalJoints() const {
    return maskToFilterMatrix(mask_rotational_joints);
}

// Reads the friction for each joint from the URDF model
void JointManager::readJointFriction(const urdf::ModelInterface& urdf_model, const pinocchio::Model& model) {
    joint_friction_ = Eigen::VectorXd::Zero(nv_);
    for (size_t i = 1; i < model.joints.size(); ++i) {
        const auto& joint = model.joints[i];
        const std::string& name = model.names[i];
        auto urdf_joint = urdf_model.getJoint(name);
        if (urdf_joint && urdf_joint->dynamics && joint.nv() == 1) {
            joint_friction_(joint.idx_v()) = urdf_joint->dynamics->friction;
        }
    }
}

// Reads the damping for each joint from the URDF model
void JointManager::readJointDamping(const urdf::ModelInterface& urdf_model, const pinocchio::Model& model) {
    joint_damping_ = Eigen::VectorXd::Zero(nv_);
    for (size_t i = 1; i < model.joints.size(); ++i) {
        const auto& joint = model.joints[i];
        const std::string& name = model.names[i];
        auto urdf_joint = urdf_model.getJoint(name);
        if (urdf_joint && urdf_joint->dynamics && joint.nv() == 1) {
            joint_damping_(joint.idx_v()) = urdf_joint->dynamics->damping;
        }
    }
}

// Reads the gear ratio for each joint from the transmissions
void JointManager::readGearRatio(const std::vector<Transmission>& transmissions, const pinocchio::Model& model) {
    gear_ratio_ = Eigen::VectorXd::Ones(nv_);
    for (const auto& transmission : transmissions) {
        if (!model.existJointName(transmission.joint_name)) continue;
        auto joint_id = model.getJointId(transmission.joint_name);
        const auto& joint = model.joints[joint_id];
        if (joint.nv() != 1) {
            throw std::invalid_argument("Joint '" + joint.shortname() + "' is complex and cannot be actuated.");
        }
        gear_ratio_(joint.idx_v()) = transmission.mechanical_reduction;
    }
}

// Reads the gear efficiency for each joint from the transmissions
void JointManager::readGearEfficiency(const std::vector<Transmission>& transmissions, const pinocchio::Model& model) {
    gear_efficiency_ = Eigen::VectorXd::Ones(nv_);
    for (const auto& transmission : transmissions) {
        if (!model.existJointName(transmission.joint_name)) continue;
        auto joint_id = model.getJointId(transmission.joint_name);
        const auto& joint = model.joints[joint_id];
        if (joint.nv() != 1) {
            throw std::invalid_argument("Joint '" + joint.shortname() + "' is complex and cannot be actuated.");
        }
        gear_efficiency_(joint.idx_v()) = transmission.efficiency;
    }
}

// Reads the gear inertia for each joint from the transmissions
void JointManager::readGearInertia(const std::vector<Transmission>& transmissions, const pinocchio::Model& model) {
    gear_inertia_ = Eigen::VectorXd::Zero(nv_);
    for (const auto& transmission : transmissions) {
        if (!model.existJointName(transmission.joint_name)) continue;
        auto joint_id = model.getJointId(transmission.joint_name);
        const auto& joint = model.joints[joint_id];
        if (joint.nv() != 1) {
            throw std::invalid_argument("Joint '" + joint.shortname() + "' is complex and cannot be actuated.");
        }
        gear_inertia_(joint.idx_v()) = transmission.inertia;
    }
}

// Calculates the mask for the joints connecting the ee with the base
void JointManager::calcMaskEeJoints(const pinocchio::Model& model, const std::string& ee_link) {
    auto joint_ids = getJointChainToLink(model, ee_link);
    mask_ee_joints = Eigen::VectorXi::Zero(nv_);
    for (auto joint_id : joint_ids) {
        const auto& j = model.joints[joint_id];
        for (int i = 0; i < j.nv(); ++i) {
            mask_ee_joints(j.idx_v() + i) = 1;
        }
    }
    mask_robot_joints = Eigen::VectorXi::Ones(nv_) - mask_ee_joints;
}

// Calculates the mask with all active joints
void JointManager::calcMaskActiveJoints(const std::vector<Transmission>& transmissions, const pinocchio::Model& model) {
    mask_active_joints = Eigen::VectorXi::Zero(nv_);
    for (const auto& transmission : transmissions) {
        if (!model.existJointName(transmission.joint_name)) continue;
        auto joint_id = model.getJointId(transmission.joint_name);
        const auto& joint = model.joints[joint_id];
        if (joint.nv() != 1) {
            throw std::invalid_argument("Joint '" + joint.shortname() + "' is complex and cannot be actuated.");
        }
        mask_active_joints(joint.idx_v()) = 1;
    }
    mask_passive_joints = Eigen::VectorXi::Ones(nv_) - mask_active_joints;
}

// Calculates the mask for the rotational joints
void JointManager::calcMaskRotationalJoints(const pinocchio::Model& model) {
    mask_rotational_joints = Eigen::VectorXi::Zero(nv_);
    for (size_t i = 1; i < model.joints.size(); ++i) { // skip universe (0)
        const auto& joint = model.joints[i];
        std::string shortname = joint.shortname();

        if (shortname == "JointModelUniverse") continue;

        if (ROTATIONAL_JOINTS.count(shortname)) {
            for (int k = 0; k < joint.nv(); ++k) mask_rotational_joints(joint.idx_v() + k) = 1;
        } else if (PRISMATIC_JOINTS.count(shortname)) {
            for (int k = 0; k < joint.nv(); ++k) mask_rotational_joints(joint.idx_v() + k) = 0;
        } else if (shortname == "JointModelFreeFlyer") {
            if (joint.nv() != 6) throw std::invalid_argument("FreeFlyerJoint should have nv=6");
            int idx = joint.idx_v();
            mask_rotational_joints(idx + 0) = 0;
            mask_rotational_joints(idx + 1) = 0;
            mask_rotational_joints(idx + 2) = 0;
            mask_rotational_joints(idx + 3) = 1;
            mask_rotational_joints(idx + 4) = 1;
            mask_rotational_joints(idx + 5) = 1;
        }
    }
}

} // namespace robot_model
