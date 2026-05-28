#include "robot_model/kinematics/levenberg_marquardt.hpp"
#include "robot_model/model/joints.hpp"
#include <iostream>
#include <cmath>

namespace robot_model {

// check if the algorithm converged
bool isConverged(const Eigen::VectorXd& h, const Eigen::VectorXd& step, double tol) {
    return (h.squaredNorm() < tol * tol) && (step.squaredNorm() < tol * tol);
}

// Levenberg-Marquardt algorithm for solving constrained optimization problems
// based on the robot model's constraints and the Jacobian of the constraints
// Todo: make the algorithm easier by implementing helper functions for the inner/outer/line_search loops
std::pair<Eigen::VectorXd, bool> levenbergMarquardt(
    RobotModel& robot_model,
    std::variant<Eigen::VectorXi, CoordinateType> mask_independent,
    const Eigen::VectorXd* q0,
    double tol,
    int max_iterations,
    int max_step_line_search)
{
    // get the mask from the independent coordinate type
    Eigen::VectorXi mask;
    if (std::holds_alternative<CoordinateType>(mask_independent)) {
        mask = robot_model.getJoints().getMask(std::get<CoordinateType>(mask_independent));
    } else {
        mask = std::get<Eigen::VectorXi>(mask_independent);
    }

    // set initial state if provided otherwise use the last known state of the model
    if (q0) {
        robot_model.updateData(robot_model.normalizeRotationalJoints(*q0));
    }

    // set inner loop maximum number of iterations as the square root of the outer loop maximum number of iterations
    int inner_max = static_cast<int>(std::ceil(std::sqrt(max_iterations)));

    // compute filter matrix that transforms from the independent coordinates to the dependent coordinates
    Eigen::VectorXi mask_dependent = Eigen::VectorXi::Ones(mask.size()) - mask;
    Eigen::MatrixXd F = maskToFilterMatrix(mask_dependent);
    Eigen::MatrixXd P = F.transpose() * F;

    // regularisation factor — prevents instability near singularities
    double lambda = 1e-3;

    // pre-allocate and initialize all temporaries outside the loop to avoid repeated heap allocations
    const int nv = robot_model.getModel().nv;
    Eigen::VectorXd step(nv);
    step.setConstant(2.0 * tol);  // initialise above tolerance
    const double step_size = 1.0/(max_step_line_search);
    Eigen::VectorXd h_current = robot_model.h();
    Eigen::VectorXd q_prev(nv);
    Eigen::MatrixXd J_filtered;
    Eigen::MatrixXd JJT;
    int outer_loop_counter = 0;
    int inner_loop_counter = 0;
    double h_norm_prev_iter = 0.0;
    double line_search_last_h_norm = 0.0;
    Eigen::VectorXd mini_step(nv);

    // iterate outer loop as long as max steps is not violated and the algorithm did not converge
    while (outer_loop_counter < max_iterations && (!isConverged(h_current, step, tol) || !robot_model.isStateValid())) {
        // set temporaries for inner loop
        inner_loop_counter = 0;
        // increase lambda with factor 10 since new random values are chosen for the independent variables
        lambda *= 10;
        h_norm_prev_iter = h_current.squaredNorm();

        // inner loop for damped least squares to calculate local solutions and check if they violate any limits
        while (inner_loop_counter < inner_max && !isConverged(h_current, step, tol)) {

            // compute filtered jacobian: J_f = J * P
            // note that only the jacobian must be computed since the data object is updated with the new state in the previous inner loop iteration
            // or initialised with the q0 vector
            robot_model.calcJacobian();
            J_filtered.noalias() = robot_model.j() * P;

            // damped least squares: step = J^T (J J^T + λI)^{-1} h
            // used since the number of constraints can be higher than the number of degrees of freedom
            JJT.noalias() = J_filtered * J_filtered.transpose();
            JJT.diagonal().array() += lambda;
            step.noalias() = J_filtered.transpose() * JJT.ldlt().solve(h_current);

            // backtracking line search with geometric step reduction
            // take the full step at first and calculate the initial values for the line search
            robot_model.updateData(robot_model.normalizeRotationalJoints(robot_model.q_motion() - step));
            h_current = robot_model.h();
            q_prev = robot_model.q_motion();
            line_search_last_h_norm = h_current.squaredNorm();
            // calculate the mini step size
            mini_step = step * step_size;
            // perform line search by taking the mini steps back to the initial state to find the optimal step size
            // optimal step size is as big as possible until the constraint error increases
            for (int ls = 0; ls < max_step_line_search; ++ls) {
                // do the mini step
                robot_model.updateData(robot_model.normalizeRotationalJoints(q_prev + mini_step));
                // calculate the current error
                h_current = robot_model.h();
                // if the current error is greater than the previous error
                // then take the previous state as the optimal state
                if (h_current.squaredNorm() > line_search_last_h_norm) {
                    robot_model.updateData(q_prev);
                    h_current = robot_model.h();
                    break;  // Sufficient decrease found — return to last step that was better
                }
                // save results from this step
                line_search_last_h_norm = h_current.squaredNorm();  
                q_prev = robot_model.q_motion();
            } // end of line search loop
            // increase inner loop counter
            inner_loop_counter++;
            // Levenberg-Marquardt adaptive damping adjustment
            if (h_current.squaredNorm() < h_norm_prev_iter) {
                lambda /= 10.0;
                lambda = std::max(lambda, 1e-9);  // Lower bound for numerical stability
            } else {
                lambda *= 10.0;
                lambda = std::min(lambda, 1e3);   // Upper bound to prevent excessive damping
            }
        } // end of inner loop

        // check state validity after each outer loop iteration
        if (!robot_model.isStateValid()) {
            // Escape joint limit violation with a targeted random perturbation
            Eigen::VectorXi violated = Eigen::VectorXi::Zero(robot_model.getJoints().getMaskRobotJoints().size());
            auto joint_limits = robot_model.checkJointLimits();
            for (int k = 0; k < joint_limits.size(); ++k) {
                if (!joint_limits(k)) violated(k) = 1;
            }
            robot_model.updateData(robot_model.createRandom(violated));
            h_current = robot_model.h();
        }
        // increase outer_loop counter by number of steps of the inner loop
        outer_loop_counter += inner_loop_counter;
    }
    // return the final state and if the algorithm converged
    return {robot_model.q_motion(), isConverged(h_current, step, tol) && robot_model.isStateValid()};
}

// calculate a valid state as motion state vector for a given configuration (UPPER or LOWER) by using the levenberg-marquardt algorithm
// the state is constrained to the given configuration by setting the end-effector (EE) position
std::pair<Eigen::VectorXd, bool> calcValidMotionState(
    RobotModel& model,
    RobotConfiguration configuration,
    double tol,
    int num_iterations,
    int max_step_line_search)
{
    // set the model to a neutral state
    Eigen::VectorXd state = model.neutralMotionConfiguration();
    // get the mask for the end-effector (EE) coordinates
    Eigen::VectorXi ee_mask = model.getJoints().getMask(CoordinateType::EE);
    // set the end-effector (EE) target values based on the configuration
    Eigen::VectorXd ee_target(6);
    if (configuration == RobotConfiguration::LOWER) {
        ee_target << 0, 0, -1, 0, 0, 0;
    } else {
        ee_target << 0, 0, 1, 0, 0, 0;
    }
    // set the end-effector (EE) target values in the state
    int j = 0;
    for (int i = 0; i < ee_mask.size(); ++i) {
        if (ee_mask(i)) {
            state(i) = ee_target(j++);
        }
    }
    // return the result by using the levenberg-marquardt algorithm to find the valid state
    return levenbergMarquardt(model, CoordinateType::ACTIVE, &state, tol, num_iterations, max_step_line_search);
}

} // namespace robot_model
