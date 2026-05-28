#include "robot_model/constraints/fixed_constraint.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/skew.hpp>
#include <stdexcept>

namespace robot_model {

/// Convert a urdf::Pose to a pinocchio SE3 transform.
static pinocchio::SE3 urdfPoseToPinocchio(const urdf::Pose& pose) {
    return pinocchio::SE3(
        Eigen::Quaterniond(
            pose.rotation.w, pose.rotation.x,
            pose.rotation.y, pose.rotation.z
        ).toRotationMatrix(),
        Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z)
    );
}

// initialize the fixed constraint by adding two OP_FRAMEs to the pinocchio model and save their ids to prevent searching when using the constraint
pinocchio::Model& FixedConstraint::initialize(pinocchio::Model& model, urdf::ConstraintSharedPtr urdf_data)
{
    setUrdfData(urdf_data);
    return initialize(model);
}

// initialize the fixed constraint by adding two OP_FRAMEs to the pinocchio model and save their ids to prevent searching when using the constraint
pinocchio::Model& FixedConstraint::initialize(pinocchio::Model& model) {

    // Check if the data object is valid
    if (!urdf_data_) {
        throw std::runtime_error("URDF data is not set");
    }
    // Validate that the links exist in the model
    if (!model.existFrame(urdf_data_->parent_link_name)) {
        throw std::runtime_error(
            "Parent link '" + urdf_data_->parent_link_name + "' not found in model");
    }
    if (!model.existFrame(urdf_data_->child_link_name)) {
        throw std::runtime_error(
            "Child link '" + urdf_data_->child_link_name + "' not found in model");
    }

    const auto parent_frame_id = model.getFrameId(urdf_data_->parent_link_name);
    const auto child_frame_id  = model.getFrameId(urdf_data_->child_link_name);

    const auto parent_joint_id = model.frames[parent_frame_id].parentJoint;
    const auto child_joint_id  = model.frames[child_frame_id].parentJoint;

    // Add OP_FRAMEs at the constraint attachment points
    const std::string p_name = urdf_data_->name + "_parent_frame";
    const std::string c_name = urdf_data_->name + "_child_frame";

    parent_id_ = model.addFrame(pinocchio::Frame(
        p_name,
        parent_joint_id,
        parent_frame_id,
        urdfPoseToPinocchio(urdf_data_->parent_to_constraint_parent_transform),
        pinocchio::FrameType::OP_FRAME
    ));

    child_id_ = model.addFrame(pinocchio::Frame(
        c_name,
        child_joint_id,
        child_frame_id,
        urdfPoseToPinocchio(urdf_data_->child_to_constraint_child_transform),
        pinocchio::FrameType::OP_FRAME
    ));

    return model;
}

// since the constraint is a fixed constraint, no projection is needed
// and an identity matrix is returned
Eigen::MatrixXd FixedConstraint::projection() const {
    return Eigen::MatrixXd::Identity(6, 6);
}

// return the dimension of the constraint by using the number of rows of the projection matrix
int FixedConstraint::dim() const {
    return static_cast<int>(projection().rows());
}

// apply the projection matrix to the full constraint equations
Eigen::VectorXd FixedConstraint::h(const pinocchio::Model& model,
                                   pinocchio::Data& data,
                                   const Eigen::VectorXd& /*q*/) const {
    return projection() * h_full(model, data);
}

// apply the projection matrix to the Jacobian of the constraint
Eigen::MatrixXd FixedConstraint::j(const pinocchio::Model& model,
                                   pinocchio::Data& data) const {
    return projection() * j_full(model, data);
}

// apply the projection matrix to the derivative of the Jacobian of the constraint
Eigen::MatrixXd FixedConstraint::jp(const pinocchio::Model& model,
                                    pinocchio::Data& data) const {
    return projection() * jp_full(model, data);
}

// calculate thedistance between the parent and child frames 
// h_full = [ p_p - p_c ;  0.5 * vee(R_rel - R_rel^T) ]
//   where R_rel = R_p^T * R_c
Eigen::VectorXd FixedConstraint::h_full(const pinocchio::Model& /*model*/,
                                        pinocchio::Data& data) const {
    const Eigen::Vector3d p_p = data.oMf[parent_id_].translation();
    const Eigen::Vector3d p_c = data.oMf[child_id_].translation();

    const Eigen::Matrix3d R_p  = data.oMf[parent_id_].rotation();
    const Eigen::Matrix3d R_c  = data.oMf[child_id_].rotation();
    const Eigen::Matrix3d R_rel = R_p.transpose() * R_c;

    // Translation error
    const Eigen::Vector3d h_T = p_p - p_c;

    // Rotation error via pinocchio::unSkew — numerically stable vee operator
    const Eigen::Vector3d h_R = 0.5 * pinocchio::unSkew(R_rel - R_rel.transpose());

    Eigen::VectorXd res(6);
    res.head<3>() = h_T;
    res.tail<3>() = h_R;
    return res;
}

// calculate the jacobian of the constraint
// J_full = [ Jp_lin - Jc_lin ;
//            0.5 * (I + R_rel) * (Jw_c - Jw_p) ]
//   where R_rel = R_p^T * R_c
Eigen::MatrixXd FixedConstraint::j_full(const pinocchio::Model& model,
                                        pinocchio::Data& data) const {
    pinocchio::Data::Matrix6x Jp = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    pinocchio::Data::Matrix6x Jc = pinocchio::Data::Matrix6x::Zero(6, model.nv);

    pinocchio::getFrameJacobian(model, data, parent_id_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jp);
    pinocchio::getFrameJacobian(model, data, child_id_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jc);

    const Eigen::Matrix3d R_p   = data.oMf[parent_id_].rotation();
    const Eigen::Matrix3d R_c   = data.oMf[child_id_].rotation();
    const Eigen::Matrix3d R_rel = R_p.transpose() * R_c;

    // Translation part
    const Eigen::MatrixXd J_T = Jp.topRows<3>() - Jc.topRows<3>();

    // Rotation part (in parent frame)
    const Eigen::MatrixXd J_rel_world = Jc.bottomRows<3>() - Jp.bottomRows<3>();
    const Eigen::MatrixXd J_rel_parent = R_p.transpose() * J_rel_world;
    const Eigen::MatrixXd J_R   =
        0.5 * (Eigen::Matrix3d::Identity() + R_rel) * J_rel_parent;

    Eigen::MatrixXd res(6, model.nv);
    res.topRows<3>()    = J_T;
    res.bottomRows<3>() = J_R;
    return res;
}

// calculate the derivative of the jacobian of the constraint
// Jp_full = [ Jp_lin_dot - Jc_lin_dot ;
//             0.5 * (R_rel_dot * J_rel + (I + R_rel) * J_rel_dot) ]
//   where R_rel_dot = R_rel * skew(omega_c - omega_p)
Eigen::MatrixXd FixedConstraint::jp_full(const pinocchio::Model& model,
                                         pinocchio::Data& data) const {
    pinocchio::Data::Matrix6x Jp     = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    pinocchio::Data::Matrix6x Jc     = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    pinocchio::Data::Matrix6x Jp_dot = pinocchio::Data::Matrix6x::Zero(6, model.nv);
    pinocchio::Data::Matrix6x Jc_dot = pinocchio::Data::Matrix6x::Zero(6, model.nv);

    pinocchio::getFrameJacobian(model, data, parent_id_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jp);
    pinocchio::getFrameJacobian(model, data, child_id_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jc);
    pinocchio::getFrameJacobianTimeVariation(model, data, parent_id_,
                                             pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jp_dot);
    pinocchio::getFrameJacobianTimeVariation(model, data, child_id_,
                                             pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jc_dot);

    const Eigen::Matrix3d R_p   = data.oMf[parent_id_].rotation();
    const Eigen::Matrix3d R_c   = data.oMf[child_id_].rotation();
    const Eigen::Matrix3d R_rel = R_p.transpose() * R_c;

    // Translation part
    const Eigen::MatrixXd J_T_dot = Jp_dot.topRows<3>() - Jc_dot.topRows<3>();

    // Relative angular-velocity Jacobian and its time derivative in parent frame
    const Eigen::MatrixXd J_rel_world     = Jc.bottomRows<3>()     - Jp.bottomRows<3>();
    const Eigen::MatrixXd J_rel_world_dot = Jc_dot.bottomRows<3>() - Jp_dot.bottomRows<3>();

    // Relative angular velocity and parent angular velocity (LOCAL_WORLD_ALIGNED)
    const pinocchio::Motion v_p =
        pinocchio::getFrameVelocity(model, data, parent_id_,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    const pinocchio::Motion v_c =
        pinocchio::getFrameVelocity(model, data, child_id_,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

    const Eigen::Vector3d omega_rel_world = v_c.angular() - v_p.angular();
    const Eigen::Vector3d omega_p_world   = v_p.angular();

    const Eigen::Vector3d omega_rel_parent = R_p.transpose() * omega_rel_world;
    const Eigen::Vector3d omega_p_parent   = R_p.transpose() * omega_p_world;

    const Eigen::Matrix3d R_rel_dot = R_rel * pinocchio::skew(omega_rel_parent);

    const Eigen::MatrixXd J_rel_parent     = R_p.transpose() * J_rel_world;
    const Eigen::MatrixXd J_rel_parent_dot = R_p.transpose() * J_rel_world_dot - 
                                             pinocchio::skew(omega_p_parent) * J_rel_parent;

    // Rotation part of Jdot
    const Eigen::MatrixXd J_R_dot =
        0.5 * (R_rel_dot * J_rel_parent +
               (Eigen::Matrix3d::Identity() + R_rel) * J_rel_parent_dot);

    Eigen::MatrixXd res(6, model.nv);
    res.topRows<3>()    = J_T_dot;
    res.bottomRows<3>() = J_R_dot;
    return res;
}

} // namespace robot_model
