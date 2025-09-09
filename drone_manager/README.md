# Drone Manager

DISCLAIMER:
This package is adapted from the trajectory_planner project by Simone D'Angelo (https://github.com/Simone-DAngelo/trajectory_planner).
The original project's structure and implementation were used as a starting point. This repository includes modifications, ROS 2 integrations and packaging changes specific to PX4/SITL use.
Please consult the original repository and its license for provenance and further attribution.

Description
-----------
`drone_manager` is a ROS 2 package that implements offboard control, motion planning, and utility nodes for PX4-based drones and SITL simulation. It groups trajectory generation, planners, frame transform helpers and a set of nodes that coordinate motion, state management and telemetry.

Purpose and responsibilities
----------------------------
- Provide a modular offboard control stack capable of receiving high-level motion requests and producing timed trajectories and low-level commands for PX4.
- Expose motion-planning components (spline-based planners and sampling-based planners) and trajectory representation used by the offboard controller.
- Offer utility nodes for state monitoring (battery, TF publishing) and manual control (teleoperation).

Main components
---------------
Nodes
- `offboard_control` — Core controller that executes trajectories, interfaces with PX4 messages, and contains trajectory and planner wiring. 
- `move_manager_node` — Higher-level movement manager that accepts motion requests (via the package's custom `MoveCmd.msg`) and orchestrates planning, acceptance checks and safe handover to the `offboard_control` node.
- `fsm_node` — Finite-state machine that manages flight modes and behaviour sequencing. It centralizes mode transitions and safety checks.
- `battery_node` — Simulates battery state and publishes alerts or status messages that other components can use to trigger safe behaviours.
- `teleop_node` — Teleoperation interface for manual control; it transforms operator inputs into `MoveCmd` or direct control messages for the offboard controller.
- `px4_tf_pub` — Publishes transforms between the PX4 vehicle frames and the ROS TF tree so other nodes can access to pose and transform data consistently.



Notes
-----
This README intentionally documents composition and responsibilities only — see individual source files and `CMakeLists.txt` for implementation details and build wiring.


FSM implementation details (from `src/fsm.cpp`)
---------------------------------------------

High-level behaviour
- The `fsm_node` implements a mission-oriented finite state machine with four main states: `Takeoff`, `Coverage`, `Land`, and `Completed`. It sequences a coverage mission over a fixed list of waypoint frames (`goal1`...`goal9`), handles low-battery interrupts by returning to a charging waypoint, and resumes the mission after a simulated recharge.

State behaviours
- Takeoff:
	- Sends a `takeoff` command on first entry.
	- Monitors odometry z and transitions to `Coverage` when within ±0.3 m of the configured takeoff altitude (1.5 m).
- Coverage:
	- Iterates through waypoints using RRT* algorithm, issuing `flyto(<waypoint>)` commands and waiting for arrival checks before advancing.
	- When all waypoints are visited, commands a return to the charging station and marks coverage complete prior to landing.
	- If `battery_low` is received, the FSM records the last visited waypoint, switches `waypoint_idx_` to the landing waypoint (index 4 — `goal5`), sends a fly-to there and transitions to `Land` when arrived.
- Land:
	- Sends `land` and, if battery was low, waits until the odometry z indicates a landed condition (z < 0.02).
	- On landing it publishes `charged_` to `/battery`, clears `battery_low_`, and either resumes the mission (by restoring the last visited waypoint and switching to `Takeoff`) or transitions to `Completed` if mission/coverage were finished.
- Completed:
	- Marks the mission as completed and logs a message; no further transitions are performed.


Assumptions and hard-coded values
- Waypoints and landing index are hardcoded in the node.
- Arrival tolerance: 0.3 m for x/y/z comparisons.
- Takeoff altitude: 1.5 m (tolerance ±0.3 m).
- Landing detection threshold: z < 0.02 m.

Notes
- The FSM expects TF frames for each waypoint to exist and be published into the `map` frame. If a transform is missing, it logs a warning and will not consider the waypoint reached.
- The `battery_node` or other code must publish `battery_low` to `/battery` for the return/land behaviour to trigger; the FSM publishes `charged_` after landing to simulate recharge.
