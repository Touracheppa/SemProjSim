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
- The IMU, 16-layer lidar and L515-mounted RGB-D camera are simulated. Camera images are ideal, aligned pinhole renderings, not a model of L515 noise or hardware calibration.
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

## RGB and depth camera

The L515 assembly has a forward-facing RGB-D sensor at an approximate optical center, just ahead of its CAD mesh. `camera_link` uses X-forward/Y-left/Z-up; `camera_optical_frame` uses Z-forward/X-right/Y-down. Both frames are in the robot TF tree. Color and depth are co-located and aligned, with shared intrinsics.

Defaults: **640 × 480, 15 Hz, 70° horizontal field of view**, depth range **0.25–9 m**. Edit the camera block in `urdf/simulation.xacro` to change these simulation settings. The mount is in `urdf/misc/Kamerabox.urdf`; neither the mount nor intrinsics are hardware-calibrated.

| ROS topic | Message / data |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image`, `rgb8` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image`, `32FC1` depth in metres |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo`, color intrinsics |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo`, aligned depth intrinsics |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2`, organized colored cloud |

All camera messages use `camera_optical_frame` and simulation timestamps. Bridges use sensor-data QoS (best effort); subscribers must use compatible QoS. Out-of-range depth pixels may be infinite, so filter non-finite values before processing.

Launch normally with `rviz:=true` to enable the RGB and depth image displays. Camera rendering also works with `gui:=false`, using the existing EGL setup. There is no additional camera node to start.

With the simulation running, validate camera streams without moving the robot:

```bash
ros2 run sieglinde_sim camera_smoke_test.py
```

This checks advancing timestamps, optical frame IDs, dimensions, RGB/depth encodings, nonempty scene data, finite metric depth, camera intrinsics and point-cloud fields.

Camera validation passed locally: all five ROS topics published advancing data at the configured resolution with the optical frame; RGB/depth samples were visually inspected, and the default obstacle measured 2.145 m at the center pixel. The RViz panel layout itself has not been visually reviewed.

## Drive and arm control node

Start the simulation as usual. In a **second terminal**, start the interactive node:

```bash
source /opt/ros/jazzy/setup.bash
source /home/lesintex/ros2_ws/src/SemProjSim/install/setup.bash
ros2 run sieglinde_sim control_node.py --ros-args -p use_sim_time:=true
```

Focus that terminal to use the keys:

| Keys | Command |
| --- | --- |
| `w` / `s` | Forward / backward |
| `a` / `d` | Turn left / right |
| Space | Stop base and hold the measured joint positions |
| `i` / `k` | Tool X + / − |
| `j` / `l` | Tool Y + / − |
| `u` / `o` | Tool Z + / − |
| `1` … `5` | Select `arm_joint_1` … `arm_joint_5` |
| `6` / `7` | Select gripper / flippers |
| `+` (or `=`) / `-` | Jog the selected joint + / − |
| `p` | Print measured tool XYZ and joint positions |
| `h` | Show help |
| `q` / Ctrl-C | Stop and exit, restoring the terminal |

Hold/repeat drive keys to keep moving. The base stops after **0.5 seconds of wall time** without a drive command, including when simulation time is paused. Default drive limits are 0.2 m/s and 0.5 rad/s; keyboard arm jogging requests 0.08 m/s in XYZ or 0.3 rad/s for a selected joint (subject to joint limits). Each drive key selects forward/backward *or* turning; send a Twist through the topic interface to combine them. Arm jogs update continuously while key repeats arrive. Keyboard input is drained every 20 ms and repeated characters refresh the current direction instead of queuing movements. Short 0.15-s trajectory lookahead with a 0.1-s braking tail stays bounded relative to measured feedback; the node sends no further jog updates while simulation time is paused.

Settings can be overridden with ROS parameters:

```bash
ros2 run sieglinde_sim control_node.py --ros-args -p use_sim_time:=true \
  -p linear_speed:=0.15 -p angular_speed:=0.4 -p xyz_jog_speed:=0.04 \
  -p joint_jog_speed:=0.2 -p motion_duration:=1.0
```

`jog_timeout` defaults to 0.2 wall-clock seconds after the last arm key repeat; on expiry the node sends a position hold. Terminals do not report key releases, and the operating system's initial key-repeat delay can still cause a brief pause after the first key press. Space stops immediately and takes priority over all buffered keys in the same input batch. `jog_horizon` defaults to 0.15 simulation seconds. The previous `xyz_step` / `joint_step` keyboard settings are replaced by the speed parameters above.

`command_timeout` and `feedback_timeout` default to 0.5 and 1.0 seconds respectively. These parameters are read at startup. `motion_duration` applies to discrete ROS topic targets, not continuous keyboard jogging. Motion duration is increased for larger discrete joint changes using the URDF velocity limits. The controller node must remain running for the drive watchdog to operate; it is not a hardware emergency stop.

### XYZ behavior

Targets describe **`tool0` in `base_link`, in metres**: X forward, Y left, Z up. `tool0` is at the center of the three gripper finger pivots (`arm_link_5` offset `0 -0.0001 0.137`). Forward kinematics, the Jacobian and inverse kinematics are derived from the installed URDF. Local damped least-squares IK starts at the measured arm position and enforces joint limits, accepting a solution within 2 mm. Unreachable targets or local solver failures are rejected with a status message; no replacement trajectory is sent.

Absolute XYZ topic commands control position only: wrist orientation is free. They produce joint-space interpolation to the endpoint, **not a straight Cartesian path or collision-aware plan**. Use small jogs in open space. The gripper fingers and flippers remain independently controllable. Commands require fresh joint feedback and a subscriber on the relevant controller topic.

### ROS topic interface

For topic-only operation (no keyboard terminal), run this **instead of** the interactive command:

```bash
ros2 launch sieglinde_sim control.launch.py
```

Run only one control node. While running, it owns `/cmd_vel` and publishes zero commands when idle; use `/control/cmd_vel` rather than publishing directly to `/cmd_vel`. Stop the control node before using other base teleop tools or the original `smoke_test.py`.

| Topic | Type / meaning |
| --- | --- |
| `/control/cmd_vel` | `geometry_msgs/msg/Twist`; clamped forward/yaw commands with timeout |
| `/control/joint_targets` | `sensor_msgs/msg/JointState`; absolute radians for any subset of the five arm joints, `Grip`, `flipper_joint` |
| `/control/xyz_target` | `geometry_msgs/msg/PointStamped`; absolute target, frame must be `base_link` |
| `/control/xyz_delta` | `geometry_msgs/msg/Vector3`; XYZ offset from current tool position in `base_link` |
| `/control/stop` | `std_msgs/msg/Empty`; stop base and hold joints with fresh feedback |
| `/control/ee_position` | `geometry_msgs/msg/PointStamped`; measured-joint FK feedback at 50 Hz |
| `/control/status` | `std_msgs/msg/String`; acceptance/rejection and control status |

Examples:

```bash
# Drive while publishing; the timeout stops the base after Ctrl-C.
ros2 topic pub -r 10 /control/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.2}}'

# Move one joint; other arm joints hold their measured positions.
ros2 topic pub --once /control/joint_targets sensor_msgs/msg/JointState \
  '{name: [arm_joint_1], position: [0.2]}'

# Jog the tool upward by 2 cm.
ros2 topic pub --once /control/xyz_delta geometry_msgs/msg/Vector3 '{z: 0.02}'

# Absolute XYZ example; reachability depends on the current arm configuration.
ros2 topic pub --once /control/xyz_target geometry_msgs/msg/PointStamped \
  '{header: {frame_id: base_link}, point: {x: 0.85, y: 0.1, z: 0.85}}'

ros2 topic pub --once /control/stop std_msgs/msg/Empty '{}'
```

Single-joint messages ignore velocity/effort arrays and use absolute positions. Unknown joints (including mimic followers), non-finite values, mismatched arrays and out-of-limit targets are rejected. Unspecified joints in an affected controller group hold their current measured position, replacing any earlier trajectory for that group.

With the simulation and exactly one control node running, `ros2 run sieglinde_sim control_smoke_test.py` tests individual-joint motion, an XYZ endpoint, unreachable-target rejection, physical driving, timeout and stop. It moves the robot, so use the supplied open world. Unit tests in `test/test_control.py` cover the analytic Jacobian, IK and input validation.

Control validation passed locally: six control unit cases plus description validation, live joint and XYZ motion, unreachable-target rejection, 0.572 m actual base displacement, command timeout and stop. Interactive terminal joint selection/jogging and Z-axis jogging were also exercised.

Continuous keyboard jogging uses damped differential kinematics for XYZ velocity, with joint-speed scaling and bounded joint-limit lookahead. It avoids the full iterative IK solve on each key repeat. Near singularities or limits, Cartesian speed/direction may be reduced or distorted; this is not collision-aware servo control. Gazebo running below real time still slows physical motion, but does not produce a backlog of keyboard commands.

Smooth-jog validation: held-key simulation test moved joint 1 by 0.159 rad, then held position after the release timeout. A 601-byte buffered input burst containing Space stopped without replaying queued movement. XYZ command output, terminal restoration, and 13 reported test results passed.
