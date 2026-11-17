# Sieglinde — ROS 2 Jazzy / Gazebo Harmonic

`SemProjSim` is an independent `ament_cmake` package named `sieglinde_sim`, ported from `../Sieglinde_2.0/sieglinde_new`. The source ROS 1 package is unchanged. All robot meshes are copied locally; the Velodyne meshes come from the Jazzy `velodyne_description` package.

## Build and run

On Ubuntu 24.04 with ROS 2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/lesintex/ros2_ws/src/SemProjSim
rosdep install --from-paths . --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch sieglinde_sim sim.launch.py
```

Build only this directory: the adjacent `sieglinde_new` package is ROS 1/catkin and cannot be built as a Jazzy package. A separate build/install directory is already present here from validation.

Options:

```bash
ros2 launch sieglinde_sim sim.launch.py rviz:=true
ros2 launch sieglinde_sim sim.launch.py gui:=false
ros2 launch sieglinde_sim sim.launch.py world:=/absolute/path/to/world.sdf
ros2 launch sieglinde_sim display.launch.py
```

The simulation starts running automatically. `gui:=false` uses headless EGL rendering for the GPU lidar; a working OpenGL/EGL driver is still required. The default world is entirely local, with a ground plane and a box obstacle. A custom world must provide Physics, UserCommands, SceneBroadcaster, Sensors and Imu systems, as in `worlds/sieglinde.sdf`.

`display.launch.py` provides the original joint-slider/RViz workflow without physics. Do not run it alongside the simulation: simulated joint states are supplied by `joint_state_broadcaster`.

## Controls and topics

| Interface | Type / purpose |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist`: forward velocity (m/s), yaw rate (rad/s) |
| `/odom`, `/tf` | Track odometry and `odom` → `base_link` transform |
| `/ground_truth/odom` | Actual simulated model pose in `world`, for validation |
| `/joint_states` | Simulated arm, gripper and flipper joint states |
| `/arm_controller/joint_trajectory` | Five arm joints, in order `arm_joint_1` … `arm_joint_5` |
| `/gripper_controller/joint_trajectory` | `Grip`; `g2` and `g3` mimic it |
| `/flipper_controller/joint_trajectory` | `flipper_joint`; `flipper_joint_mimic` follows at 0.46× |
| `/scan`, `/points` | Lidar scan and 16-layer point cloud, frame `velodyne` |
| `/imu/data` | IMU, frame `imu` |
| `/clock` | Simulation clock |

Each trajectory controller also exposes a `follow_joint_trajectory` action. Joint commands use radians and respect the original URDF limits.

Drive forward, then explicitly stop (the track plugin retains its last command):

```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

Move the arm:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5], points: [{positions: [0.15, 0.1, -0.1, 0.1, 0.1], time_from_start: {sec: 3}}]}'
```

Move flippers or gripper:

```bash
ros2 topic pub --once /flipper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [flipper_joint], points: [{positions: [0.2], time_from_start: {sec: 3}}]}'
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [Grip], points: [{positions: [0.2], time_from_start: {sec: 3}}]}'
```

## Porting decisions and fidelity

- ROS 1 launches, catkin, transmissions and `libgazebo_ros_control.so` are replaced with ROS 2 launch, ament, `gz_ros2_control` and trajectory controllers. Mimic joints have state interfaces only.
- The old `hector_xacro_tools` dependency is replaced with a local cuboid inertia macro. The unused VLP-16 macro include is removed; its existing visual meshes are retained.
- Original chassis, arm, gripper, camera assembly, transforms and joint limits are retained. Sensor joints were renamed `imu_joint` and `velodyne_joint` to avoid duplicate SDF frame names.
- Original track collisions are moved onto six small-mass fixed links, preserving their geometry and articulation. Harmonic's native `TrackedVehicle` and `TrackController` systems drive contact surfaces. The fixed track joints are preserved during SDF conversion. This models rigid belts, not individual chain links. Speed is limited to ±0.6 m/s per belt and steering efficiency is an approximate 0.5.
- Gripper/wrist Coulomb friction is reduced from 5 to 0.01 N·m for simulation: the original gripper friction exceeded its 4 N·m effort limit and prevented motion.
- The source arm/gripper tensors (`0.002, 0.02, 0.002`) violate the inertia triangle inequality. They are replaced with box estimates using the original mass/COM and approximate link dimensions. Box/IMU links receive missing inertia. These are simulation estimates, not calibrated hardware dynamics.
- The IMU and 16-layer lidar are simulated. The L515/camera assembly retains its original geometry; RGB/depth camera streams are not implemented.
- RViz settings are migrated from the supplied `sieglinde.rviz` to ROS 2 plugin names and a transient-local robot-description topic.
- Source copyright notices are retained. The upstream package declares its license as `TODO` and some descriptions carry proprietary notices; no new license is asserted for those assets.

## Validation

```bash
colcon test --packages-select sieglinde_sim --event-handlers console_direct+
colcon test-result --verbose
# With the simulation running, in a second sourced terminal:
ros2 run sieglinde_sim smoke_test.py
```

The description test checks mesh resolution, the link tree, physically valid inertia tensors and mimic interfaces. The live smoke test commands the robot and checks clock, IMU, point cloud, joint/mimic tracking and actual model displacement (independent of track odometry). Run it in the supplied open world with room ahead; it moves the robot and stops it afterward.

Implementation references: [Jazzy gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) and the installed Harmonic `tracked_vehicle_simple.sdf` example under `gz_sim_vendor`.

Validated locally on Jazzy / Gazebo Sim 8.15.0: build and description test passed; URDF-to-SDF conversion succeeded; all four controllers activated; live smoke test passed with 1.160 m of actual base displacement. GUI appearance has not been visually reviewed.

Harmonic's DART backend may log that native SDF mimic constraints are unsupported. Here `gz_ros2_control` implements mimic tracking instead; all three follower joints are checked by the live test. `gz_frame_id` extension warnings during SDF conversion are expected. The IMU uses `ros_gz_bridge`, not a ros2_control sensor interface.
