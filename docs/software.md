# Software

[Back to the README](../README.md)

## Software stack

| Layer | Technology | Documentation |
|-------|------------|---------------|
| Compute | NVIDIA Jetson Orin Nano, Docker + NVIDIA runtime | [Jetson Orin Nano](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/index.html), [Docker](https://docs.docker.com/), [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/) |
| Middleware | ROS 2 Lyrical (inside the `isaac_ros_dev` container on the car) | [ROS 2 Lyrical docs](https://docs.ros.org/en/lyrical/) |
| Perception / SLAM | NVIDIA Isaac ROS: **cuVSLAM** (visual odometry/SLAM), stereo depth, **nvblox** (3D map) | [Isaac ROS](https://nvidia-isaac-ros.github.io/), [Visual SLAM (cuVSLAM)](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html), [nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) |
| Planning / control | **Nav2** with an Ackermann behavior tree (no spin/back-up recovery; slow, blind reversing is allowed), `ackermann_cmd_node` converts Nav2 velocity commands to the ESP's `/cmd_vel` and `/steer_cmd` | [Nav2](https://docs.nav2.org/) |
| Motor / steering controller | ESP32 running a **micro-ROS** client (UDP 8888); the micro-ROS agent on the car bridges it into ROS 2 | [micro-ROS](https://micro.vulcanexus.org/) |
| Cameras | Zero-copy stereo camera node (Argus), calibration included, optional rear USB camera | [sensor_msgs (Image, CameraInfo)](https://docs.ros.org/en/lyrical/p/sensor_msgs/) |
| Vehicle model | URDF generated from one config (`vehicle.yaml`), frames follow REP-105 (`base_link` = centre of the rear axle, x forward, y left, z up) | [URDF tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html), [REP-105 (frames)](https://reps.openrobotics.org/rep-0105/) |
| Visualization | Foxglove, RViz2 (Docker), Jetson health via `/diagnostics` | [Foxglove](https://docs.foxglove.dev/), [RViz2](https://docs.ros.org/en/lyrical/p/rviz2/) |

## Topics

Topics are available while the corresponding part of the pipeline runs (`full` = SLAM + depth + nvblox,
`nav` = `full` + Nav2). ESP topics only appear when the ESP32 is connected to the micro-ROS agent.
Camera and map topics are large; use the `/preview/...` topics over Wi-Fi.

### Sensors

| Topic | Type | Description |
|-------|------|-------------|
| `/visual_slam/image_0`, `/visual_slam/image_1` | `sensor_msgs/Image` | Raw left / right stereo images (60 Hz) |
| `/visual_slam/camera_info_0`, `/visual_slam/camera_info_1` | `sensor_msgs/CameraInfo` | Stereo calibration |
| `/left/image_rect`, `/right/image_rect` (+ `camera_info_rect`) | `sensor_msgs/Image` | Rectified stereo images |
| `/preview/left/compressed`, `/preview/right/compressed` | `sensor_msgs/CompressedImage` | Low-rate JPEG previews for remote viewing |
| `/back_camera/image/compressed` | `sensor_msgs/CompressedImage` | Rear USB camera (only if connected) |
| `/stereo/skew_ms` | `std_msgs/Float32` | Time skew between the stereo images in ms |
| `/disparity`, `/depth` | `stereo_msgs/DisparityImage`, `sensor_msgs/Image` | Stereo depth |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | cuVSLAM visual odometry |
| `/visual_slam/tracking/vo_path`, `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | Visual odometry path / loop-closed SLAM path |
| `/visual_slam/status`, `/slam_status` | `VisualSlamStatus`, `std_msgs/UInt8` | Tracking state |
| `/visual_slam/vis/landmarks_cloud`, `/visual_slam/vis/observations_cloud` | `sensor_msgs/PointCloud2` | Tracked landmarks / features |
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | nvblox 3D map (ESDF) |
| `/nvblox_node/static_occupancy_grid` | `nav_msgs/OccupancyGrid` | nvblox 2D occupancy grid |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | nvblox 3D mesh (heavy) |
| `/nvblox_node/mesh_marker` | `visualization_msgs/Marker` | The same mesh for RViz (heavy: the whole mesh is re-published on every update) |
| `/tf`, `/tf_static`, `/robot_description` | | Transforms and vehicle model (URDF) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Jetson health (CPU/GPU load, temperature, power) |
| `/steer_state` | `car_interfaces/SteerState` | Steering angle (rad, + left) and calibration state (20 Hz) |
| `/controller_state`, `/controller_raw` | `car_interfaces/ControllerState`, `sensor_msgs/Joy` | Remote controller connection and raw axes/buttons |
| `/lights_state` | `car_interfaces/LightsState` | Lights as actually shown by the firmware |
| `/wheel_odom`, `/batt_status` | `car_interfaces/WheelOdom`, `BattStatus` | Defined in the contract, **not published yet** (no hardware) |

### Navigation (Nav2)

| Topic | Type | Description |
|-------|------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Send a navigation goal (Foxglove: goal pose tool) |
| `/navigate_to_pose` (action) | `nav2_msgs/action/NavigateToPose` | Send a goal and get feedback and result; used by [examples/goal.py](../examples/goal.py) |
| `/navigate_to_pose/_action/cancel_goal` (service) | `action_msgs/srv/CancelGoal` | **Cancel navigation**: stops the active goal. From code call `cancel_goal_async()` on the goal handle (Ctrl-C does this in `goal.py`); from a shell: `ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"` (an empty request cancels all goals) |
| `/plan`, `/plan_smoothed` | `nav_msgs/Path` | Global plan |
| `/global_costmap/costmap`, `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Costmaps |
| `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` | Vehicle footprint |
| `/cmd_vel_nav`, `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | Raw / smoothed velocity command from Nav2 |
| `/nav_debug/lethal_local`, `/nav_debug/lethal_global` | `nav_msgs/OccupancyGrid` | Only the real obstacle cells of the costmaps, without the inflation halo |
| `/nav_debug/markers`, `/nav_debug/status` | `visualization_msgs/MarkerArray`, `std_msgs/String` | Debug overlays: distance rings from the front bumper, camera field of view, footprint collision check, goal distance, and the last Nav2 planner/controller message (also as text) |

### Actuators

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Speed (m/s) command to the ESP; `ackermann_cmd_node` forwards Nav2's smoothed velocity here, the ESP maps it to drive percent |
| `/drive_cmd` | `std_msgs/Float32` | Drive in percent of the controller stick range, -100..100 (+ forward) |
| `/steer_cmd` | `car_interfaces/SteerCmd` | Target steering angle in rad (+ left); from Nav2 via the bicycle model in `ackermann_cmd_node` |
| `/drive_mode` | `car_interfaces/DriveMode` | `1` = remote controller, `2` = autonomy; set by the car's pipeline, students do not publish it |
| `/lights_cmd` | `car_interfaces/LightsCmd` | Front lamps and rear bars: colours, indicators, hazard, brake/reverse flags |
| `/horn_cmd` | `std_msgs/Bool` | Horn |
| `/jetson_heartbeat` | `std_msgs/Header` | 1 Hz heartbeat; autonomy stops if it is older than 3 s |

The ESP32 interface (units, QoS, firmware behaviour) is defined in the contract in the main project repository
(`ESP Contract/CONTRACT.md`). The car only drives autonomously in drive mode `2`, and the remote controller
always takes over.

### Car administration

For when nobody can ssh into the car. These are available while the pipeline runs in `nav` mode.

| Topic / service | Type | Description |
|-----------------|------|-------------|
| `/control_room/restart_pipeline` (service) | `std_srvs/srv/Trigger` | **Restart the whole pipeline** (cameras, SLAM, nvblox, Nav2): `ros2 service call /control_room/restart_pipeline std_srvs/srv/Trigger`. It is back after about 1-2 minutes; a second request within a minute is refused |
| `/control_room/set_time` | `builtin_interfaces/Time` | **Set the car's clock** (Unix seconds, only `sec` is used): `ros2 topic pub --once /control_room/set_time builtin_interfaces/msg/Time "{sec: $(date +%s)}"`. The car restarts its pipeline afterwards because the clock jump breaks SLAM. Easier: [scripts/set_car_time.py](../scripts/set_car_time.py). Times far from 2026 or within 5 s of the car's clock are ignored |
| `/control_room/status` | `std_msgs/String` | Answer to the last request (`ros2 topic echo /control_room/status`) |

The car's clock matters: after a power cycle it starts in 1970 and transforms and goals fail until it is set (see
"Clock skew" in [TROUBLESHOOTING.md](../TROUBLESHOOTING.md)). `scripts/doctor.py` shows the difference to your laptop.
