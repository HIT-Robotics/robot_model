#include "robot_model/model/robot_model.hpp"
#include "robot_model/constraints/constraint_manager.hpp"
#include "robot_model/model/limits.hpp"
#include "robot_model/kinematics/levenberg_marquardt.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <optional>

namespace py = pybind11;
using namespace robot_model;

// function to export RobotModel as python package
void exportRobotModel(py::module_ &m) {
  // coordinate type
  py::enum_<CoordinateType>(m, "CoordinateType")
      .value("ROTATIONAL", CoordinateType::ROTATIONAL)
      .value("ACTIVE", CoordinateType::ACTIVE)
      .value("PASSIVE", CoordinateType::PASSIVE)
      .value("EE", CoordinateType::EE)
      .value("ROBOT", CoordinateType::ROBOT)
      .export_values();

  // robot configurations
  py::enum_<RobotConfiguration>(m, "RobotConfiguration")
      .value("UPPER", RobotConfiguration::UPPER)
      .value("LOWER", RobotConfiguration::LOWER)
      .export_values();

  // joint manager
  py::class_<JointManager>(m, "Joints")
      .def_property_readonly("active_joints", &JointManager::getMaskActiveJoints)
      .def_property_readonly("passive_joints", &JointManager::getMaskPassiveJoints)
      .def_property_readonly("end_effector_joints", &JointManager::getMaskEeJoints)
      .def_property_readonly("robot_joints", &JointManager::getMaskRobotJoints)
      .def_property_readonly("rotation_joints", &JointManager::getMaskRotationalJoints)
      .def_property_readonly("filter_matrix_active_joints", &JointManager::getFilterMatrixActiveJoints)
      .def_property_readonly("filter_matrix_passive_joints", &JointManager::getFilterMatrixPassiveJoints)
      .def_property_readonly("filter_matrix_endeffector_joints", &JointManager::getFilterMatrixEeJoints)
      .def_property_readonly("filter_matrix_robot_joints", &JointManager::getFilterMatrixRobotJoints)
      .def_property_readonly("filter_matrix_rotation_joints", &JointManager::getFilterMatrixRotationalJoints)
      .def("getMask", py::overload_cast<CoordinateType>(&JointManager::getMask, py::const_), py::arg("coord"))
      .def("getMask", py::overload_cast<const std::vector<std::string>&>(&JointManager::getMask, py::const_), py::arg("joint_names"));

  // constraint manager
  py::class_<ConstraintManager>(m, "Constraints")
      .def("dim", &ConstraintManager::dim);
    
  // limit manager
  py::class_<LimitManager>(m, "Limits");

  // robot model
  py::class_<RobotModel>(m, "RobotModel")
      .def(py::init<const std::string &>())
      .def_property_readonly("joints", [](RobotModel &self) { return &self.getJoints(); }, py::return_value_policy::reference_internal)
      .def_property_readonly("constraints", [](RobotModel &self) { return &self.getConstraints(); }, py::return_value_policy::reference_internal)
      .def_property_readonly("limits", [](RobotModel &self) { return &self.getLimits(); }, py::return_value_policy::reference_internal)
      .def("checkLinkLimits", [](RobotModel &self, const Eigen::VectorXd &q) {
          auto mask = self.checkJointLimits(&q);
          return (mask.array() != 0).all();
      })
      .def("calcInverseDynamicsTree", [](RobotModel &self, const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &qp,
                                         const Eigen::VectorXd &qpp) {
          return self.h(&q, &qp, &qpp);
      }, py::arg("q"), py::arg("qp"), py::arg("qpp"))
      .def("h", [](RobotModel &self, std::optional<Eigen::VectorXd> q, std::optional<Eigen::VectorXd> qp, std::optional<Eigen::VectorXd> qpp) {
          return self.h(q ? &(*q) : nullptr, qp ? &(*qp) : nullptr, qpp ? &(*qpp) : nullptr);
      }, py::arg("q") = py::none(), py::arg("qp") = py::none(), py::arg("qpp") = py::none())
      .def("j", [](RobotModel &self, std::optional<Eigen::VectorXd> q, std::optional<Eigen::VectorXd> qp, std::optional<Eigen::VectorXd> qpp) {
          return self.j(q ? &(*q) : nullptr, qp ? &(*qp) : nullptr, qpp ? &(*qpp) : nullptr);
      }, py::arg("q") = py::none(), py::arg("qp") = py::none(), py::arg("qpp") = py::none())
      .def("j_d", [](RobotModel &self, std::optional<Eigen::VectorXd> q, std::optional<Eigen::VectorXd> qp, std::optional<Eigen::VectorXd> qpp) {
          return self.j_d(q ? &(*q) : nullptr, qp ? &(*qp) : nullptr, qpp ? &(*qpp) : nullptr);
      }, py::arg("q") = py::none(), py::arg("qp") = py::none(), py::arg("qpp") = py::none())
      .def_property_readonly("degrees_of_freedom", &RobotModel::degrees_of_freedom)
      .def("checkJointLimits", [](RobotModel &self, std::optional<Eigen::VectorXd> q, std::optional<Eigen::VectorXd> qp, std::optional<Eigen::VectorXd> qpp) {
          return self.checkJointLimits(q ? &(*q) : nullptr, qp ? &(*qp) : nullptr, qpp ? &(*qpp) : nullptr);
      }, py::arg("q") = py::none(), py::arg("qp") = py::none(), py::arg("qpp") = py::none())
      .def("isStateValid", [](RobotModel &self, std::optional<Eigen::VectorXd> q, std::optional<Eigen::VectorXd> qp, std::optional<Eigen::VectorXd> qpp) {
          return self.isStateValid(q ? &(*q) : nullptr, qp ? &(*qp) : nullptr, qpp ? &(*qpp) : nullptr);
      }, py::arg("q") = py::none(), py::arg("qp") = py::none(), py::arg("qpp") = py::none())
      .def("normalizeRotationalJoints", &RobotModel::normalizeRotationalJoints)
      .def("neutralStateConfiguration", &RobotModel::neutralStateConfiguration)
      .def("neutralMotionConfiguration", &RobotModel::neutralMotionConfiguration)
      .def("randomStateConfiguration", &RobotModel::randomStateConfiguration)
      .def("randomMotionConfiguration", &RobotModel::randomMotionConfiguration)
      .def("createRandom", [](RobotModel &self, const Eigen::VectorXi &random_mask, std::optional<Eigen::VectorXd> q) {
          return self.createRandom(random_mask, q ? &(*q) : nullptr);
      }, py::arg("random_mask"), py::arg("q") = py::none())
      .def("q_motion", [](RobotModel &self) { return self.q_motion(); })
      .def("q_state", [](RobotModel &self) { return self.q_state(); })
      .def("motionToStateJoint", &RobotModel::motionToStateJoint)
      .def("stateToMotionJoint", &RobotModel::stateToMotionJoint)
      .def("updateData", [](RobotModel &self, const Eigen::VectorXd& q, std::optional<Eigen::VectorXd> qp, std::optional<Eigen::VectorXd> qpp, bool calc_jacobian, bool calc_jacobian_time_derivative) {
          self.updateData(q, qp ? &(*qp) : nullptr, qpp ? &(*qpp) : nullptr, calc_jacobian, calc_jacobian_time_derivative);
      }, py::arg("q"), py::arg("qp") = py::none(), py::arg("qpp") = py::none(), 
         py::arg("calc_jacobian") = false, py::arg("calc_jacobian_time_derivative") = false)
      .def_property_readonly("joint_names", [](RobotModel &self) {
          const auto &names = self.getModel().names;
          // names[0] is the universe joint, skip it
          return std::vector<std::string>(names.begin() + 1, names.end());
      })
      .def_property_readonly("njoints", [](RobotModel &self) {
          return self.getModel().njoints;
      });

  // levenberg-marquardt
  m.def("levenbergMarquardt",
        [](RobotModel& rm,
           std::variant<Eigen::VectorXi, CoordinateType> mask,
           std::optional<Eigen::VectorXd> q0,
           double tol, int max_iterations, int max_step_line_search) {
            return robot_model::levenbergMarquardt(
                rm, mask, q0 ? &(*q0) : nullptr, tol, max_iterations, max_step_line_search);
        },
        py::arg("robot_model"), py::arg("mask_independent"), py::arg("q0") = py::none(),
        py::arg("tol") = 1e-6, py::arg("max_iterations") = 50, py::arg("max_step_line_search") = 10);
  
        // calc valid motion state    
  m.def("calcValidMotionState",
        [](RobotModel& model, RobotConfiguration configuration,
           double tol, int num_iterations, int max_step_line_search) {
            return robot_model::calcValidMotionState(
                model, configuration, tol, num_iterations, max_step_line_search);
        },
        py::arg("model"), py::arg("configuration") = RobotConfiguration::LOWER,
        py::arg("tol") = 1e-10, py::arg("num_iterations") = 100, py::arg("max_step_line_search") = 10);
}

PYBIND11_MODULE(robot_model, m) {
  exportRobotModel(m);
}
