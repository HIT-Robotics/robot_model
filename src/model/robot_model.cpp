#include "robot_model/model/robot_model.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <urdf_parser/urdf_parser.h>
#include <tinyxml2.h>
#include <regex>
#include <stdexcept>

namespace robot_model {

// constructor
RobotModel::RobotModel(const std::string& path)
    : jacobian_valid_(false), jacobian_time_valid_(false)
{
    std::cout << "RobotModel: Loading path: " << path << std::endl;
    // Check file extension
    std::regex re(".+\\.[au]rdf$", std::regex_constants::icase);
    if (!std::regex_match(path, re)) {
        throw std::runtime_error("Expected to get an ardf or urdf file as robot description");
    }

    // Generate urdf model
    std::cout << "RobotModel: Parsing URDF..." << std::endl;
    urdf_model_ = urdf::parseURDFFile(path);
    if (!urdf_model_) {
        throw std::runtime_error("Failed to parse URDF file: " + path);
    }

    // Generate pinocchio model
    std::cout << "RobotModel: Building Pinocchio model..." << std::endl;
    try {
        pinocchio::urdf::buildModel(path, model_);
    } catch (const std::exception& e) {
        std::cerr << "RobotModel: Pinocchio failed: " << e.what() << std::endl;
        throw;
    }
    
    // Parse transmissions (Option C: Manual XML parsing)
    std::cout << "RobotModel: Parsing transmissions..." << std::endl;
    auto transmissions = parseTransmissions(path);

    // Generate constraints and get updated model
    std::cout << "RobotModel: Initializing constraints..." << std::endl;
    constraints_ = std::make_unique<ConstraintManager>();
    model_ = constraints_->initialize(model_, *urdf_model_);

    // Generate limits
    std::cout << "RobotModel: Initializing limits..." << std::endl;
    limits_ = std::make_unique<LimitManager>(*urdf_model_);

    // Generate joint data
    std::cout << "RobotModel: Initializing joints..." << std::endl;
    joints_ = std::make_unique<JointManager>(model_, *urdf_model_, transmissions);

    // Generate data object
    data_ = std::make_unique<pinocchio::Data>(model_);

    // Update data with zero configuration
    std::cout << "RobotModel: Updating data..." << std::endl;
    updateData(Eigen::VectorXd::Zero(model_.nv));
    std::cout << "RobotModel: Construction finished." << std::endl;
}

// parse the transmission from the URDF file
std::vector<Transmission> RobotModel::parseTransmissions(const std::string& path) {
    std::vector<Transmission> transmissions;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return transmissions;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) return transmissions;

    for (tinyxml2::XMLElement* trans_elem = root->FirstChildElement("transmission"); trans_elem; trans_elem = trans_elem->NextSiblingElement("transmission")) {
        Transmission trans;
        
        // Find joint name
        tinyxml2::XMLElement* joint_elem = trans_elem->FirstChildElement("joint");
        if (joint_elem && joint_elem->Attribute("name")) {
            trans.joint_name = joint_elem->Attribute("name");
        } else {
            // Some URDFs might have multiple joints or different structure, but we expect one for our parallel robot logic
            continue;
        }

        // Find actuator params
        tinyxml2::XMLElement* actuator_elem = trans_elem->FirstChildElement("actuator");
        if (actuator_elem) {
            tinyxml2::XMLElement* red_elem = actuator_elem->FirstChildElement("mechanicalReduction");
            trans.mechanical_reduction = red_elem ? red_elem->DoubleText(1.0) : 1.0;

            tinyxml2::XMLElement* eff_elem = actuator_elem->FirstChildElement("efficiency");
            trans.efficiency = eff_elem ? eff_elem->DoubleText(1.0) : 1.0;

            tinyxml2::XMLElement* ine_elem = actuator_elem->FirstChildElement("inertia");
            trans.inertia = ine_elem ? ine_elem->DoubleText(0.0) : 0.0;
        } else {
            trans.mechanical_reduction = 1.0;
            trans.efficiency = 1.0;
            trans.inertia = 0.0;
        }
        transmissions.push_back(trans);
    }
    return transmissions;
}

// update the data with the given joint configuration, velocity and acceleration
void RobotModel::updateData(const Eigen::VectorXd& q, 
                            const Eigen::VectorXd* qp, 
                            const Eigen::VectorXd* qpp, 
                            bool calc_jacobian, 
                            bool calc_jacobian_time_derivative)
{
    // check whether the joint configuration is in the operational space (nv) or configuration space (nq)
    if (q.size() == model_.nv) {
        q_motion_ = q;
        q_state_ = motionToStateJoint(q);
    } else if (q.size() == model_.nq) {
        q_state_ = q;
        q_motion_ = stateToMotionJoint(q);
    } else {
        throw std::runtime_error("vector q expects to be size (" + std::to_string(model_.nq) + ",) or (" + std::to_string(model_.nv) + ",) but is " + std::to_string(q.size()));
    }

    // check if joint velocities are given, otherwise set them to zero
    if (qp) {
        qp_ = *qp;
    } else {
        qp_ = Eigen::VectorXd::Zero(model_.nv);
    }

    // check if joint accelerations are given, otherwise set them to zero
    if (qpp) {
        qpp_ = *qpp;
    } else {
        qpp_ = Eigen::VectorXd::Zero(model_.nv);
    }
    
    // reset the jacobian validity flags
    jacobian_valid_ = false;
    jacobian_time_valid_ = false;

    // compute forward kinematics
    pinocchio::forwardKinematics(model_, *data_, q_state_, qp_, qpp_);
    // update the frame placements
    pinocchio::updateFramePlacements(model_, *data_);

    // compute jacobians if needed
    if (calc_jacobian || calc_jacobian_time_derivative) {
        calcJacobian();
    }

    // compute jacobian time derivative if needed
    if (calc_jacobian_time_derivative) {
        calcJacobianTimeVariation();
    }
}

// computes the jacobians of all joints
void RobotModel::calcJacobian() {
    jacobian_valid_ = true;
    pinocchio::computeJointJacobians(model_, *data_, q_state_);
}

// computes the time derivative of the jacobians of all joints
void RobotModel::calcJacobianTimeVariation(const Eigen::VectorXd* qp) {
    if (qp) {
        qp_ = *qp;
    }
    jacobian_time_valid_ = true;
    pinocchio::computeJointJacobiansTimeVariation(model_, *data_, q_state_, qp_);
}

// convert motion vector to state joint vector by integrating from neutral state
Eigen::VectorXd RobotModel::motionToStateJoint(const Eigen::VectorXd& q) const {
    return pinocchio::integrate(model_, pinocchio::neutral(model_), q);
}

// convert state joint vector to motion vector by computing difference from neutral state
Eigen::VectorXd RobotModel::stateToMotionJoint(const Eigen::VectorXd& q) const {
    return pinocchio::difference(model_, pinocchio::neutral(model_), q);
}

// compute the constraint error
Eigen::VectorXd RobotModel::h(const Eigen::VectorXd* q, const Eigen::VectorXd* qp, const Eigen::VectorXd* qpp) {
    if (q) {
        updateData(*q, qp, qpp);
    }
    return constraints_->h(model_, *data_, q_motion_);
}

// compute the constraint jacobian
Eigen::MatrixXd RobotModel::j(const Eigen::VectorXd* q, const Eigen::VectorXd* qp, const Eigen::VectorXd* qpp) {
    if (q) {
        updateData(*q, qp, qpp, true);
    }
    if (!jacobian_valid_) {
        throw std::runtime_error("jacobian is invalid. Make sure to update the jacobian before trying to calculate the jacobian");
    }
    return constraints_->j(model_, *data_);
}

// compute the constraint jacobian time derivative
Eigen::MatrixXd RobotModel::j_d(const Eigen::VectorXd* q, const Eigen::VectorXd* qp, const Eigen::VectorXd* qpp) {
    if (q) {
        updateData(*q, qp, qpp, true, true);
    }
    if (!jacobian_valid_) {
        throw std::runtime_error("jacobian is invalid. Make sure to update the jacobian before trying to calculate the jacobian time derivative");
    }
    if (!jacobian_time_valid_) {
        throw std::runtime_error("jacobian time derivative is invalid. Make sure to update the jacobian before trying to calculate the jacobian time derivative");
    }
    return constraints_->jp(model_, *data_);
}

// add tooling to the robot model
void RobotModel::addTooling(const pinocchio::Inertia& inertia, const pinocchio::SE3& center_of_gravity, const std::string& joint) {
    pinocchio::JointIndex joint_id;
    if (joint.empty()) {
        auto body_id = model_.getFrameId("tool0");
        joint_id = model_.frames[body_id].parentJoint;
    } else {
        joint_id = model_.getJointId(joint);
    }
    model_.appendBodyToJoint(joint_id, inertia, center_of_gravity);
    data_ = std::make_unique<pinocchio::Data>(model_);
    updateData(q_motion_, &qp_, &qpp_);
}

// checks the joint limits for the given joint configuration
Eigen::VectorXi RobotModel::checkJointLimits(const Eigen::VectorXd* q, const Eigen::VectorXd* qp, const Eigen::VectorXd* qpp) {
    if (q) {
        updateData(*q, qp, qpp);
    }
    return limits_->checkJointLimits(model_, q_motion_);
}

// checks if the state is valid
bool RobotModel::isStateValid(const Eigen::VectorXd* q, const Eigen::VectorXd* qp, const Eigen::VectorXd* qpp) {
    if (q) {
        updateData(*q, qp, qpp);
    }
    return limits_->stateValid(model_, *data_, q_motion_);
}

// normalize the rotational joints to the range [-pi, pi]
Eigen::VectorXd RobotModel::normalizeRotationalJoints(const Eigen::VectorXd& q) const {
    const auto& mask = joints_->getMaskRotationalJoints();
    Eigen::VectorXd q_normalized = q;
    for (int i = 0; i < mask.size(); ++i) {
        if (mask(i)) {
            q_normalized(i) = std::fmod(q_normalized(i) + M_PI, 2.0 * M_PI);
            if (q_normalized(i) < 0) q_normalized(i) += 2.0 * M_PI;
            q_normalized(i) -= M_PI;
        }
    }
    return q_normalized;
}

// neutral state configuration
Eigen::VectorXd RobotModel::neutralStateConfiguration() const {
    return pinocchio::neutral(model_);
}

// neutral operational space configuration
Eigen::VectorXd RobotModel::neutralMotionConfiguration() const {
    return normalizeRotationalJoints(stateToMotionJoint(neutralStateConfiguration()));
}

// random state configuration
Eigen::VectorXd RobotModel::randomStateConfiguration() const {
    return pinocchio::randomConfiguration(model_);
}

// random operational space configuration
Eigen::VectorXd RobotModel::randomMotionConfiguration() const {
    return normalizeRotationalJoints(stateToMotionJoint(randomStateConfiguration()));
}

// xreate a random state by only changing the masked dof's
Eigen::VectorXd RobotModel::createRandom(const Eigen::VectorXi& random_mask, const Eigen::VectorXd* q) {
    Eigen::VectorXd q_res;
    if (q) {
        q_res = *q;
    } else {
        q_res = q_motion_;
    }

    Eigen::VectorXd rand_config;
    if (random_mask.size() == model_.nv) {
        rand_config = randomMotionConfiguration();
    } else {
        rand_config = randomStateConfiguration();
    }

    for (int i = 0; i < random_mask.size(); ++i) {
        if (random_mask(i)) {
            q_res(i) = rand_config(i);
        }
    }
    return normalizeRotationalJoints(q_res);
}

} // namespace robot_model
