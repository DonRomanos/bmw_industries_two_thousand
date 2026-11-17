# BMW Industries Two Thousand

BITT (a.k.a. **KITT**) is a self-driving toy car: a 12 V ride-on BMW that perceives its surroundings with a
stereo camera, builds a map, localizes itself and drives autonomously. This kit contains everything you need to
**talk to the car from your own laptop** and to **replay recorded ROS 2 bags** without the car.

What is in here:

| Path | Content |
|------|---------|
| [foxglove/bmw_kitt.layout.json](foxglove/bmw_kitt.layout.json) | Foxglove layout: navigation view, 3D SLAM view, cameras, diagnostics, ESP state, buttons for lights/horn/drive |
| [docker/rviz/](docker/rviz) | RViz2 containers: live view of the car (`run.sh`) and bag replay (`Dockerfile.replay`) |
| [scripts/view_bag.sh](scripts/view_bag.sh) | Replay a recorded bag in RViz2 |
| [bags/](bags) | Put your rosbag2 directories here (contents are git-ignored) |

A simulation will be added later.

## Hardware stack

_To be filled out._

## Software stack

| Layer | Technology |
|-------|------------|
| Compute | NVIDIA Jetson Orin Nano, Docker + NVIDIA runtime |
| Middleware | ROS 2 Lyrical (inside the `isaac_ros_dev` container on the car) |
| Perception / SLAM | NVIDIA Isaac ROS: **cuVSLAM** (visual odometry/SLAM), stereo depth, **nvblox** (3D map) |
| Planning / control | **Nav2** with a forward-only Ackermann behavior tree, `ackermann_cmd_node` converts velocity commands to Ackermann commands |
| Motor / steering controller | ESP32 running a **micro-ROS** client (UDP 8888); the micro-ROS agent on the car bridges it into ROS 2 |
| Cameras | Zero-copy stereo camera node (Argus), calibration included, optional rear USB camera |
| Vehicle model | URDF generated from one config (`vehicle.yaml`), frames follow REP-105 (`base_link` = centre of the rear axle, x forward, y left, z up) |
| Visualization | Foxglove, RViz2 (Docker), Jetson health via `/diagnostics` |

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
| `/plan`, `/plan_smoothed` | `nav_msgs/Path` | Global plan |
| `/global_costmap/costmap`, `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Costmaps |
| `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` | Vehicle footprint |
| `/cmd_vel_nav`, `/cmd_vel_smoothed` | `geometry_msgs/TwistStamped` | Raw / smoothed velocity command from Nav2 |

### Actuators

| Topic | Type | Description |
|-------|------|-------------|
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Speed and steering angle from Nav2 (`ackermann_cmd_node`) |
| `/drive_cmd` | `std_msgs/Float32` | Drive in percent of the controller stick range, -100..100 (+ forward) |
| `/steer_cmd` | `car_interfaces/SteerCmd` | Target steering angle in rad (+ left) |
| `/cmd_vel` | `geometry_msgs/Twist` | Speed (m/s) and yaw rate (rad/s) command to the ESP |
| `/drive_mode` | `car_interfaces/DriveMode` | `1` = remote controller, `2` = autonomy |
| `/lights_cmd` | `car_interfaces/LightsCmd` | Front lamps and rear bars: colours, indicators, hazard, brake/reverse flags |
| `/horn_cmd` | `std_msgs/Bool` | Horn |
| `/jetson_heartbeat` | `std_msgs/Header` | 1 Hz heartbeat; autonomy stops if it is older than 3 s |

The ESP32 interface (units, QoS, firmware behaviour) is defined in the contract in the main project repository
(`ESP Contract/CONTRACT.md`). The car only drives autonomously in drive mode `2`, and the remote controller
always takes over.

## Connecting to the car

The car opens its own Wi-Fi access point on boot (no internet access through it).

| | |
|---|---|
| SSID | `KITT` |
| Password | `hasselhoff` |
| Car address | `10.42.0.1` (your laptop gets `10.42.0.x` via DHCP) |
| Foxglove bridge | `ws://10.42.0.1:8765` |
| micro-ROS agent (ESP32) | UDP `8888` |

Check with `ping 10.42.0.1`.

## Foxglove (recommended)

1. Open [Foxglove](https://app.foxglove.dev) (web) or the desktop app.
2. *Open connection → Foxglove WebSocket* → `ws://10.42.0.1:8765`.
3. *Layouts → Import from file…* → [foxglove/bmw_kitt.layout.json](foxglove/bmw_kitt.layout.json).

The layout has a top-down **Navigation** tab (costmaps, plan, footprint, velocity plot) and a 3D SLAM/nvblox tab,
both with a goal pose tool that publishes `/goal_pose`, plus left/right/back cameras, diagnostics, ESP state
(steering, controller, lights) and buttons to publish lights, horn, drive mode, drive and steering.

## RViz2 in Docker

Requires Docker. The helper scripts are currently written for **Windows with WSL2 + WSLg**; on native Linux you
have to adapt the X11/Wayland options in the `docker run` calls.

### Live view of the car

```bash
docker/rviz/run.sh [CAR_IP]        # default 10.42.0.1, builds the image on first use
```

The image contains RViz2 with the nvblox and cuVSLAM plugins. DDS multicast is unreliable over Wi-Fi, so the car is
added as a static discovery peer. Set `GPU=1` to use the WSL GPU for rendering.

### Replay a recorded bag

Copy a rosbag2 (MCAP) directory into [bags/](bags), then:

```bash
scripts/view_bag.sh bags/<name> [--rate 2] [--no-images] [--loop] [--partial] [--no-cloud]
```

The bag is replayed in an isolated DDS domain (it does not touch the car's ROS graph). RViz2 shows the robot and
TF moving, the complete recorded path, the camera images with feature tracks, a dashboard (speed, stereo skew,
tracking time) and the recorded nvblox map. Without a bag argument the newest bag in `bags/` is used.
`--partial` replays only the complete files of a bag that was not copied completely.

## Roadmap

- Simulation of the car (robot model and Nav2 setup are already exported for it)
- Hardware description
- One Docker image for all nodes
