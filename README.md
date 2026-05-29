# robot_model

<p align="center">
  <img src="docs/logos/autonox_Robotics_Logo.png" height="60" style="margin-right: 40px;">
  <img src="docs/logos/hhn_logo.png" height="80">
</p>
The **robot_model** package provides extended URDF-based robot modeling with support for closed-loop kinematics via constraint definitions.

It enables computing a full, valid joint configuration from a subset of joint values while satisfying kinematic constraints.

## Key Idea

Define constraints directly in the URDF -> solve dependent joints automatically using a Levenberg–Marquardt approach.

## Features

- Support for **closed-loop kinematics**
- Constraint-based joint completion
- Integration with **Pinocchio** for tree kinematics and dynamics
- Python API via **pybind11**
- Access to:
  - joints (active, passive, end-effector)
  - constraints
  - limits
  - kinematics & Jacobians

## Supported Constraints

- Fixed (cut link)
- Revolute (using parent axis)
- Spherical (no axes)

## Architecture

- **URDF (extended)** -> parsed via custom `urdfdom`
- **Pinocchio** -> tree kinematics & dynamics
- **RobotModel class** -> constraint-aware abstraction layer
- **levenbergMarquardt function** -> calculate a state that satisfies the constraint based on a minimal set of joint values, e.g. Endeffector position or measured active joint values, using a Levenberg Marquardt Algorithm with a line-search to find a feasible step-size for each iteration

## Dependencies

You must use the modified versions:

- urdfdom_headers: <https://github.com/HIT-Robotics/urdfdom_headers>
- urdfdom: <https://github.com/HIT-Robotics/urdfdom>

Pinocchio must be rebuilt against the modified `urdfdom`.

- pinocchio 3.9: <https://github.com/stack-of-tasks/pinocchio/tree/v3.9.0> (higher versions lead to errors when extracting joints and their ids)

## Installation

1. Add all required repositories to your workspace:
   - `urdfdom_headers`
   - `urdfdom`
   - `pinocchio`
   - `robot_model`
2. Build:

   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

Use the package as a library.
Main entry point:

- RobotModel class

Provides:

- Tree representation via Pinocchio
- Constraint handling
- Limit checking
- State Handling between motion state of the URDF and pinocchio states
- Closed-loop kinematics solving

## Roadmap

- Upstream URDF extensions into official urdfdom
- Add support for additional constraint types

## Support
<Fabian.Finkbeiner@hs-heilbronn.de>

## Acknowledgment

Developed by the Robotics Lab at Heilbronn University of Applied Sciences

Supported by autonox Robotics GmbH
