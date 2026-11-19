# BMW Industries Two Thousand

BITT (a.k.a. **KITT**) is a self-driving toy car: a 12 V ride-on BMW that perceives its surroundings with a
stereo camera, builds a map, localizes itself and drives autonomously. This kit contains everything you need to
**talk to the car from your own laptop** and to **replay recorded ROS 2 bags** without the car.

What is in here:

| Path | Content |
|------|---------|
| [.devcontainer/](.devcontainer), [docker/student/](docker/student) | Development container with ROS 2 and the car's message types, preconfigured to reach the car |
| [student_ws/](student_ws) | Your colcon workspace (put your packages in `student_ws/src`) |
| [rviz/kitt.rviz](rviz/kitt.rviz), [scripts/rviz.sh](scripts/rviz.sh) | RViz2 layout and launcher for the dev container |
| [examples/](examples) | Example scripts: camera, obstacles, navigation goals, lights, bag reader |
| [scripts/doctor.py](scripts/doctor.py), [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Connection check for the car and fixes for common problems |
| [scripts/play_bag.sh](scripts/play_bag.sh), [scripts/kitt_env.sh](scripts/kitt_env.sh) | Replay a bag as the car inside the dev container / switch a terminal between car and replay |
| [ros_ws/src/car_interfaces/](ros_ws/src/car_interfaces) | The car's message definitions (lights, steering, drive mode, ...), built into the dev container |
| [foxglove/bmw_kitt.layout.json](foxglove/bmw_kitt.layout.json) | Foxglove layout: navigation view, 3D SLAM view, cameras, diagnostics, ESP state, buttons for lights/horn/drive |
| [docker/rviz/](docker/rviz) | Bag replay assets (RViz2 layout, path/cloud/visualisation helpers) used by `scripts/view_bag.sh` |
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
| Planning / control | **Nav2** with an Ackermann behavior tree (no spin/back-up recovery; slow, blind reversing is allowed), `ackermann_cmd_node` converts Nav2 velocity commands to the ESP's `/cmd_vel` and `/steer_cmd` |
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

## Connecting to the car

The car opens its own Wi-Fi access point on boot (no internet access through it).

| | |
|---|---|
| SSID | `KITT` |
| Password | `********` |
| Car address | `10.42.0.1` (your laptop gets `10.42.0.x` via DHCP) |
| Foxglove bridge | `ws://10.42.0.1:8765` |
| micro-ROS agent (ESP32) | UDP `8888` |

Check with `ping 10.42.0.1`.

## Development container

The quickest way to write code for the car. Requires Docker and VS Code with the *Dev Containers* extension.

1. Open this folder in VS Code and run *Dev Containers: Reopen in Container* (the first build needs internet).
2. Connect your laptop to the `KITT` Wi-Fi.
3. In a terminal inside the container:

```bash
ros2 topic list                       # the car's topics should show up
ros2 topic echo /steer_state
ros2 topic hz /preview/left/compressed
```

The container uses host networking and Fast DDS with the car as a static discovery peer
(`ROS_STATIC_PEERS=10.42.0.1`, `ROS_DOMAIN_ID=0`). To use another address or domain, set `KITT_IP` /
`ROS_DOMAIN_ID` in your host environment before opening the container. Host networking works on Linux and on
Windows with Docker running inside WSL2; Docker Desktop is not supported.

Your packages go into `student_ws/src`; it is built once on container creation (`colcon build` afterwards) and
sourced automatically in new terminals. The `car_interfaces` messages are already available:

```python
from car_interfaces.msg import LightsCmd, DriveMode
```

Run `scripts/doctor.py` to check the connection to the car step by step. If `ros2 topic list` stays empty, see
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## Examples

Small, commented scripts in [examples/](examples) to copy from (run them inside the dev container, no build needed):

| Script | What it shows |
|--------|---------------|
| [camera.py](examples/camera.py) | Subscribe to the camera preview, decode the JPEG with OpenCV, save the latest frame to `examples/out/frame.jpg` |
| [obstacles.py](examples/obstacles.py) | Read Nav2's local costmap and print the nearest obstacle ahead / left / right in metres |
| [goal.py](examples/goal.py) | Send a navigation goal (`python3 examples/goal.py 2.0 0.5 --yaw 90`) and follow its progress |
| [lights.py](examples/lights.py) | Headlights, indicators, hazard, brake/reverse, any colour, horn |

The car's pipeline switches it to autonomy by itself while Nav2 is driving, and the remote controller always takes
over, so keep it within reach when you send goals. `camera.py` needs the cameras running and `obstacles.py`/`goal.py` the `nav` pipeline on the car.

### Working with recorded bags

Put a rosbag2 (MCAP) directory into [bags/](bags). Two ways to use it inside the dev container:

**Replay it as if it were the car** – all examples and your own nodes then run unchanged:

```bash
scripts/play_bag.sh bags/<name> --loop --topics /tf /tf_static /preview/left/compressed   # terminal 1
source scripts/kitt_env.sh replay                                                          # terminal 2
python3 examples/camera.py
source scripts/kitt_env.sh live                                                            # back to the car
```

The bag is published in an isolated DDS domain (77, shared memory inside the container only), so replaying can never reach the car. Extra
arguments go to `ros2 bag play` (`--rate 0.5`, `--start-offset 30`, ...). Only topics that were recorded exist, and
the raw stereo pair is heavy, so play just what you need with `--topics`.

**Read it offline from Python** – [bag_reader.py](examples/bag_reader.py) needs no running ROS graph:

```bash
python3 examples/bag_reader.py info   bags/<name>                          # topics, types, counts, rates
python3 examples/bag_reader.py echo   bags/<name> /steer_state -n 3
python3 examples/bag_reader.py images bags/<name> /preview/left/compressed --every 30
python3 examples/bag_reader.py odom   bags/<name> --out examples/out/odom.csv
```

and in your own code: `from bag_reader import read_messages, image_to_bgr`.

For a ready-made RViz bag viewer on the host see [RViz2 in Docker](#rviz2-in-docker-host-without-the-dev-container).

## Foxglove (recommended)

1. Open [Foxglove](https://app.foxglove.dev) (web) or the desktop app.
2. *Open connection → Foxglove WebSocket* → `ws://10.42.0.1:8765`.
3. *Layouts → Import from file…* → [foxglove/bmw_kitt.layout.json](foxglove/bmw_kitt.layout.json).

The layout has a top-down **Navigation** tab (costmaps, plan, footprint, velocity plot) and a 3D SLAM/nvblox tab,
both with a goal pose tool that publishes `/goal_pose`, plus left/right/rear cameras, diagnostics, ESP state
(steering, controller, lights) and buttons to publish lights, horn, drive mode, drive and steering.

## RViz2

RViz2 is installed in the dev container and shows the same things as Foxglove (costmaps, plan, footprint, SLAM
path, camera preview, goal tool) with the layout in [rviz/kitt.rviz](rviz/kitt.rviz):

```bash
scripts/rviz.sh
```

Pick **2D Goal Pose** in the toolbar and click on the map to send a goal (the car drives when Nav2 gets a goal; keep the
remote controller within reach). The nvblox displays and TF are off by default; tick them in the Displays panel.
`File → Save Config` writes your changes back to `rviz/kitt.rviz`. In a terminal switched to a bag replay
(`source scripts/kitt_env.sh replay`) RViz shows the replay instead of the car.

The window opens on your host through the X11 socket (`/tmp/.X11-unix`, `DISPLAY`), rendered in software:

- **Windows (WSL2):** works out of the box (WSLg).
- **Linux:** run `xhost +local:` on the host once per login if RViz cannot connect to the display.
- If `DISPLAY` is empty in the container, start VS Code from a terminal where `echo $DISPLAY` shows a value.

## RViz2 in Docker (host, without the dev container)

Requires Docker. The helper script is currently written for **Windows with WSL2 + WSLg**; on native Linux you
have to adapt the X11/Wayland options in its `docker run` call. It uses the same image as the dev container
([docker/student/Dockerfile](docker/student/Dockerfile)), so there is only one image to build. To see the live car,
use the dev container and `scripts/rviz.sh` (see [RViz2](#rviz2)).

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
