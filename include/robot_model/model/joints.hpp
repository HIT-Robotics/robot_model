#ifndef ROBOT_MODEL_MODEL_JOINTS_HPP
#define ROBOT_MODEL_MODEL_JOINTS_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <pinocchio/multibody/model.hpp>
#include <urdf_model/model.h>

namespace robot_model {

// struct to store transmission information for each joint
struct Transmission {
    std::string joint_name;
    double mechanical_reduction;
    double efficiency;
    double inertia;
};

// enum class to define the coordinate types to get a specigfic filter matrix
enum class CoordinateType {
    ROTATIONAL, // all rotational joints
    ACTIVE, // all active joints
    PASSIVE, // all passive joints
    EE, // all joints connecting the ee with the robots base
    ROBOT // all joints except the ones connecting the ee with the robot base
};

// convert a mask to a filter matrix
Eigen::MatrixXd maskToFilterMatrix(const Eigen::VectorXi& mask);

// class to manage the joints of the robot
class JointManager {
public:
    JointManager(const pinocchio::Model& model, 
                 const urdf::ModelInterface& urdf_model,
                 const std::vector<Transmission>& transmissions,
                 const std::string& ee_link = "tool0");

    const Eigen::VectorXd& getGearInertia() const { return gear_inertia_; }
    const Eigen::VectorXd& getGearRatio() const { return gear_ratio_; }
    const Eigen::VectorXd& getGearEfficiency() const { return gear_efficiency_; }
    const Eigen::VectorXd& getJointFriction() const { return joint_friction_; }
    const Eigen::VectorXd& getJointDamping() const { return joint_damping_; }

    const Eigen::VectorXi& getMaskEeJoints() const { return mask_ee_joints; }
    const Eigen::VectorXi& getMaskRobotJoints() const { return mask_robot_joints; }
    const Eigen::VectorXi& getMaskActiveJoints() const { return mask_active_joints; }
    const Eigen::VectorXi& getMaskPassiveJoints() const { return mask_passive_joints; }
    const Eigen::VectorXi& getMaskRotationalJoints() const { return mask_rotational_joints; }

    Eigen::VectorXi getMask(CoordinateType coord) const;
    Eigen::VectorXi getMask(const std::vector<std::string>& joint_names) const;

    Eigen::MatrixXd getFilterMatrixEeJoints() const;
    Eigen::MatrixXd getFilterMatrixRobotJoints() const;
    Eigen::MatrixXd getFilterMatrixActiveJoints() const;
    Eigen::MatrixXd getFilterMatrixPassiveJoints() const;
    Eigen::MatrixXd getFilterMatrixRotationalJoints() const;

private:
    int nv_;
    Eigen::VectorXd gear_inertia_;
    Eigen::VectorXd gear_ratio_;
    Eigen::VectorXd gear_efficiency_;
    Eigen::VectorXd joint_friction_;
    Eigen::VectorXd joint_damping_;    
    Eigen::VectorXi mask_ee_joints;
    Eigen::VectorXi mask_robot_joints;
    Eigen::VectorXi mask_active_joints;
    Eigen::VectorXi mask_passive_joints;
    Eigen::VectorXi mask_rotational_joints;

    std::unordered_map<std::string, std::pair<int, int>> joint_info_;

    void calcMaskEeJoints(const pinocchio::Model& model, const std::string& ee_link);
    void calcMaskRotationalJoints(const pinocchio::Model& model);
    void calcMaskActiveJoints(const std::vector<Transmission>& transmissions, const pinocchio::Model& model);
    void readGearRatio(const std::vector<Transmission>& transmissions, const pinocchio::Model& model);
    void readGearEfficiency(const std::vector<Transmission>& transmissions, const pinocchio::Model& model);
    void readGearInertia(const std::vector<Transmission>& transmissions, const pinocchio::Model& model);
    void readJointFriction(const urdf::ModelInterface& urdf_model, const pinocchio::Model& model);
    void readJointDamping(const urdf::ModelInterface& urdf_model, const pinocchio::Model& model);
};

} // namespace robot_model

#endif // ROBOT_MODEL_MODEL_JOINTS_HPP
