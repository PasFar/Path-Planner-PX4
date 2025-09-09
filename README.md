# Planning and Navigation Project

## Setup Instructions

1. **Clone the support utilities repository:**

   ```bash
   git clone https://github.com/Prisma-Drone-Team/sitl_utils.git
   ```

2. **Clone the PX4 Autopilot repository:**

   ```bash
   cd sitl_utils
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   ```

3. **Clone the custom Drone Manager package and the custom ArUco package:**

   ```bash
   cd ..
   mkdir -p ros2_ws-src/pkg
   cd ros2_ws-src/pkg
   git clone https://github.com/PasFar/Drone-Manager.git
   ```

4. **Set up custom simulation assets and configuration:**

   Replace `/PATH/TO/` with the absolute path to your `sitl_utils` folder:

   ```bash
   cp /PATH/TO/sitl_utils/ros2_ws-src/pkg/models/leo_race_field /PATH/TO/sitl_utils/PX4-Autopilot/Tools/simulation/gz/models/ -r

   cp /PATH/TO/sitl_utils/ros2_ws-src/pkg/models/default.sdf /PATH/TO/sitl_utils/PX4-Autopilot/Tools/simulation/gz/worlds/

   cp /PATH/TO/sitl_utils/ros2_ws-src/pkg/model.sdf /PATH/TO/sitl_utils/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/ 
   
   ```

5. **Build the Docker image:**

   ```bash
   cd /PATH/TO/sitl_utils/docker
   docker build -t leo-img -f px4_humble_dockerfile.txt .
   ```

6. **Run the Docker container:**

   ```bash
   cd /PATH/TO/sitl_utils
   ./run_cnt.sh
   ```

7. **Setup PX4 developmnent enviromnent**

    ```bash
   bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
   ```
   

---

## Simulation Instructions

Open the container and enter in `ros2_ws`:
```bash
cd ros2_ws
colcon build
source install/setup.bash
```
Then launch the whole simulation with:
```bash
cd src/pkg/drone_manager
tmuxp load simulation.yml
```
This will open the following terminals

Terminals opened by `tmuxp load simulation.yml` and purpose
---------------------------------------------------------
The `simulation.yml` session defines two named tmux windows (`px4` and `rtabmap`) each split into multiple panes. Below is a concise mapping of each pane and why it is opened.

Window: `px4`
- Pane 1: `cd ~/PX4-Autopilot` + `make px4_sitl gz_x500_depth`
   - Starts (or rebuilds and starts) the PX4 SITL binary using the `gz_x500_depth` Gazebo world. This runs the autopilot simulation instance the rest of the stack connects to.
- Pane 2: `MicroXRCEAgent udp4 -p 8888`
   - Runs the Micro XRCE Agent which provides a communication bridge (typically between PX4 and external middleware or micro-ROS devices) over UDP.
- Pane 3: `ros2 run ros_gz_bridge parameter_bridge ...`
   - Launches the `ros_gz_bridge` parameter bridge to forward selected Gazebo topics (camera, depth, odometry, /clock) into ROS 2 topics so ROS nodes can consume simulated sensors and simulation time.
- Pane 4: `ros2 launch drone_manager rtabmap_sim.launch.py`
   - Starts the RTAB-Map-related launch which may provide mapping/localization utilities used in the simulation.
- Pane 5: `ros2 run joy joy_node --ros-args -r use_sim_time:=true`
   - Runs a joystick input node (used for teleoperation). In simulation this can be left active to accept human inputs or mapped test inputs.
- Pane 6: `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 run rviz2 rviz2 -d .../rviz/leo.rviz`
   - Opens RViz configured with the provided `leo.rviz` file for live visualization of the robot, point clouds, maps and trajectories. `LD_PRELOAD` ensures RViz can load octomap plugins if needed.
- Pane 7: `ros2 launch drone_manager tf_static_sim.launch.py use_sim_time:=true`
   - Publishes static transforms and TF configuration required for simulation frames (helps keep the TF tree consistent for planners and visualizers).

Window: `rtabmap`
- Pane 1: `ros2 run drone_manager offboard_control --ros-args --params-file .../sim_params.yaml`
   - Starts the main offboard controller node which executes trajectories and sends commands to PX4.
- Pane 2: `ros2 run drone_manager move_manager_node --ros-args --params-file .../sim_params.yaml`
   - Starts the movement manager which accepts `MoveCmd` requests, triggers planners and coordinates handoff to the offboard controller.
- Pane 3: `ros2 topic echo /leo/drone/plan_status`
   - A simple monitor to observe planning status messages (useful to see plan acceptance, failure or progress in real time).
- Pane 4: `ros2 run drone_manager battery_node`
   - Runs the battery monitor node which publishes battery status used by safety logic.
- Pane 5: `ros2 launch drone_manager px4_tf_pub_simulation.launch.py use_sim_time:=true`
   - Launches a node that continuously publishes PX4-related TFs into ROS 2 for proper frame relationships in simulation.
- Pane 6: `ros2 run drone_manager fsm_node`
   - Starts the finite-state machine node which manages flight modes and high-level behaviour sequencing.
- Pane 7: `ros2 launch aruco_ros multi.launch.py marker_ids:=201,1,2 marker_sizes:=0.5,0.5,0.5`
   - Starts ArUco marker publishers/detection nodes used for visual localization and testing of marker-based perception.

Notes
- The tmuxp session staggers startup with `sleep` between panes to ensure dependencies are available when each node starts.
- Use the tmux keybindings (or `tmuxp` controls) to focus a pane, restart a single node, or view logs. Many panes are long-running processes; if a node exits, check its output in the pane to diagnose the issue.
- There are commented example `ros2 topic pub` commands in `simulation.yml` that demonstrate how to trigger actions (takeoff, go, flyto, teleop, land) manually; these are useful for testing once the session is running.


