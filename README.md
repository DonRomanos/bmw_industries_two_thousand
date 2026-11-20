<h1 align="center">🚗 BMW Industries Two Thousand</h1>

<p align="center"><em>He's coming: BITT. A car. A computer. A toy. BITT. A car that drives itself fights the chaos in the lecture hall.</em></p>

<p align="center"><img src="docs/kitt-scanner.svg" alt="Animated red KITT-style scanner light" width="400"></p>

<p align="center">
  <img alt="ROS 2 Lyrical" src="https://img.shields.io/badge/ROS%202-Lyrical-22314E?logo=ros&logoColor=white">
  <img alt="Jetson Orin Nano" src="https://img.shields.io/badge/NVIDIA-Jetson%20Orin%20Nano-76B900?logo=nvidia&logoColor=white">
  <img alt="Isaac ROS" src="https://img.shields.io/badge/Isaac%20ROS-cuVSLAM%20%7C%20nvblox-76B900?logo=nvidia&logoColor=white">
  <img alt="Nav2" src="https://img.shields.io/badge/Nav2-Ackermann-0A7BBB">
  <img alt="micro-ROS" src="https://img.shields.io/badge/micro--ROS-ESP32--S3-E7352C">
</p>

<p align="center">
  <a href="#-connecting-to-the-car">Connect</a> ·
  <a href="#-development-container">Dev container</a> ·
  <a href="#-examples">Examples</a> ·
  <a href="#-foxglove-recommended">Foxglove</a> ·
  <a href="#-rviz2">RViz2</a> ·
  <a href="#-bag-viewer">Bags</a> ·
  <a href="#-simulation">Simulation</a> ·
  <a href="docs/hardware.md">Hardware</a> ·
  <a href="docs/software.md">Software</a> ·
  <a href="TROUBLESHOOTING.md">Troubleshooting</a>
</p>

---

BITT is a self-driving toy car: a 12 V ride-on BMW that perceives its surroundings with a
stereo camera, builds a map, localizes itself and drives autonomously. This kit contains everything you need to
**talk to the car from your own laptop** and to **replay recorded ROS 2 bags** without the car.

> [!TIP]
> New here? Connect to the `BITT` Wi-Fi, open the [dev container](#-development-container), run `scripts/doctor.py`, then try an [example](#-examples).

## 📂 What is in here

<details open>
<summary>Repository map</summary>

| Path | Content |
|------|---------|
| [.devcontainer/](.devcontainer) | Development container with ROS 2 and the car's message types, preconfigured to reach the car |
| [student_ws/](student_ws) | Your colcon workspace (put your packages in `student_ws/src`) |
| [rviz/bitt.rviz](rviz/bitt.rviz), [scripts/rviz.sh](scripts/rviz.sh), [scripts/controls.py](scripts/controls.py) | RViz2 layout (shared by the live view and the bag viewer), launcher for the dev container, and the lights/horn/cancel button window |
| [examples/](examples) | Example scripts: camera, obstacles, navigation goals, lights, bag reader |
| [scripts/doctor.py](scripts/doctor.py), [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Connection check for the car and fixes for common problems |
| [scripts/set_car_time.py](scripts/set_car_time.py) | Set the car's clock from your laptop over ROS (the car has no battery-backed clock) |
| [scripts/play_bag.sh](scripts/play_bag.sh), [scripts/bitt_env.sh](scripts/bitt_env.sh) | Replay a bag as the car inside the dev container / switch a terminal between car and replay |
| [ros_ws/src/car_interfaces/](ros_ws/src/car_interfaces) | The car's message definitions (lights, steering, drive mode, ...), built into the dev container |
| [foxglove/bmw_bitt.layout.json](foxglove/bmw_bitt.layout.json) | Foxglove layout: navigation view, 3D SLAM view, cameras, diagnostics, ESP state, buttons for lights/horn/drive |
| [scripts/view_bag/](scripts/view_bag) | Bag replay helpers (path/cloud/visualisation, layout switcher) used by `scripts/view_bag.sh` |
| [scripts/view_bag.sh](scripts/view_bag.sh) | Replay a recorded bag in RViz2 |
| [ros_ws/src/bitt_sim/](ros_ws/src/bitt_sim), [scenarios/](scenarios) | Gazebo simulation of the car with its real Nav2 setup, and the world files (`.sdf`) it runs in (contents git-ignored) |
| [bags/](bags) | Put your rosbag2 directories here (contents are git-ignored) |

</details>

## 🔧 Hardware

The car is a 12 V ride-on BMW i8 Concept: a Jetson runs perception and navigation, an ESP32-S3 drives the motors, lights and horn.
Wiring overview, components, controller mapping and steering calibration: **[docs/hardware.md](docs/hardware.md)**.

## 💻 Software

ROS 2 Lyrical with Isaac ROS (cuVSLAM, nvblox), Nav2 and a micro-ROS firmware on the ESP32.
Software stack and the full topic reference (sensors, navigation, actuators): **[docs/software.md](docs/software.md)**.

## 📡 Connecting to the car

The car opens its own Wi-Fi access point on boot (no internet access through it).

| | |
|---|---|
| SSID | `BITT` |
| Password | `********` (ask your instructors) |
| Car address | `10.42.0.1` (your laptop gets `10.42.0.x` via DHCP) |
| Foxglove bridge | `ws://10.42.0.1:8765` |
| micro-ROS agent (ESP32) | UDP `8888` |

Check with `ping 10.42.0.1`.

## 🐳 Development container

The quickest way to write code for the car. Requires Docker and VS Code with the *Dev Containers* extension.

1. Open this folder in VS Code and run *Dev Containers: Reopen in Container* (the first build needs internet).
2. Connect your laptop to the `BITT` Wi-Fi.
3. In a terminal inside the container:

```bash
ros2 topic list                       # the car's topics should show up
ros2 topic echo /steer_state
ros2 topic hz /preview/left/compressed
```

The container uses host networking and Fast DDS with the car as a static discovery peer
(`ROS_STATIC_PEERS=10.42.0.1`, `ROS_DOMAIN_ID=0`). To use another address or domain, set `BITT_IP` /
`ROS_DOMAIN_ID` in your host environment before opening the container. Host networking works on Linux and on
Windows with Docker running inside WSL2; Docker Desktop is not supported.

Your packages go into `student_ws/src`; it is built once on container creation (`colcon build` afterwards) and
sourced automatically in new terminals. The `car_interfaces` messages are already available:

```python
from car_interfaces.msg import LightsCmd, DriveMode
```

Run `scripts/doctor.py` to check the connection to the car step by step. If `ros2 topic list` stays empty, see
[TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## 🧪 Examples

Small, commented scripts in [examples/](examples) to copy from (run them inside the dev container, no build needed):

| Script | What it shows |
|--------|---------------|
| [camera.py](examples/camera.py) | Subscribe to the camera preview, decode the JPEG with OpenCV, save the latest frame to `examples/out/frame.jpg` |
| [obstacles.py](examples/obstacles.py) | Read Nav2's local costmap and print the nearest obstacle ahead / left / right in metres |
| [goal.py](examples/goal.py) | Send a navigation goal (`python3 examples/goal.py 2.0 0.5 --yaw 90`) and follow its progress |
| [lights.py](examples/lights.py) | Headlights, indicators, hazard, brake/reverse, any colour, horn |

> [!WARNING]
> The car's pipeline switches it to autonomy by itself while Nav2 is driving, and the remote controller always takes
> over, so keep it within reach when you send goals. `camera.py` needs the cameras running and `obstacles.py`/`goal.py` the `nav` pipeline on the car.

### Working with recorded bags

Put a rosbag2 (MCAP) directory into [bags/](bags). Two ways to use it inside the dev container:

**Replay it as if it were the car** – all examples and your own nodes then run unchanged:

```bash
scripts/play_bag.sh bags/<name> --loop --topics /tf /tf_static /preview/left/compressed   # terminal 1
source scripts/bitt_env.sh replay                                                          # terminal 2
python3 examples/camera.py
source scripts/bitt_env.sh live                                                            # back to the car
```

> [!IMPORTANT]
> The bag is published in an isolated DDS domain (77, shared memory inside the container only), so replaying can never reach the car. Extra
> arguments go to `ros2 bag play` (`--rate 0.5`, `--start-offset 30`, ...). Only topics that were recorded exist, and
> the raw stereo pair is heavy, so play just what you need with `--topics`.

**Read it offline from Python** – [bag_reader.py](examples/bag_reader.py) needs no running ROS graph:

```bash
python3 examples/bag_reader.py info   bags/<name>                          # topics, types, counts, rates
python3 examples/bag_reader.py echo   bags/<name> /steer_state -n 3
python3 examples/bag_reader.py images bags/<name> /preview/left/compressed --every 30
python3 examples/bag_reader.py odom   bags/<name> --out examples/out/odom.csv
```

and in your own code: `from bag_reader import read_messages, image_to_bgr`.

For a ready-made RViz bag viewer see [Bag viewer](#-bag-viewer).

## 🦊 Foxglove (recommended)

1. Open [Foxglove](https://app.foxglove.dev) (web) or the desktop app.
2. *Open connection → Foxglove WebSocket* → `ws://10.42.0.1:8765`.
3. *Layouts → Import from file…* → [foxglove/bmw_bitt.layout.json](foxglove/bmw_bitt.layout.json).

The layout has a top-down **Navigation** tab (costmaps, plan, footprint, velocity plot) and a 3D SLAM/nvblox tab,
both with a goal pose tool that publishes `/goal_pose`, plus left/right/rear cameras, diagnostics, ESP state
(steering, controller, lights) and buttons to publish lights, horn, drive mode, drive and steering.

## 🖥️ RViz2

RViz2 is installed in the dev container and shows the same things as Foxglove (costmaps, plan, footprint, SLAM
path, camera preview, goal tool) with the layout in [rviz/bitt.rviz](rviz/bitt.rviz):

```bash
scripts/rviz.sh
```

Pick **2D Goal Pose** in the toolbar and click on the map to send a goal (the car drives when Nav2 gets a goal; keep the
remote controller within reach). The **Cameras**, **nvblox** and **Bag replay** groups and TF are off by default (each
camera opens a big docked panel that squeezes the Displays list, and nvblox and stereo images are heavy over Wi-Fi); tick
them in the Displays panel. The cuVSLAM landmarks are off because there are
thousands of them; *Panels → Views* has a saved 3D overview.
RViz works on a temporary copy of the layout (it rewrites the file it was started with when you close it), so closing it
never changes the repository; to keep layout changes use `File → Save Config As…` and pick `rviz/bitt.rviz`. RViz has no buttons of its own, so `scripts/rviz.sh`
also opens a small **BITT controls** window ([scripts/controls.py](scripts/controls.py), always on top) with buttons for headlights,
all off, left/right indicators, hazard, horn (sounds while held) and **Cancel navigation** (cancels the active Nav2
goal); `NO_CONTROLS=1 scripts/rviz.sh` skips it. In a terminal switched to a bag replay
(`source scripts/bitt_env.sh replay`) RViz shows the replay instead of the car.

The window opens on your host through the X11 socket (`/tmp/.X11-unix`, `DISPLAY`), rendered in software:

- **Windows (WSL2):** works out of the box (WSLg).
- **Linux:** run `xhost +local:` on the host once per login if RViz cannot connect to the display.
- If `DISPLAY` is empty in the container, start VS Code from a terminal where `echo $DISPLAY` shows a value.

## 🎞️ Bag viewer

`scripts/view_bag.sh` replays a bag in RViz2 and works in two places:

- **In the dev container:** runs directly (RViz opens through the X11 socket, see [RViz2](#-rviz2)).
- **On the host with Docker** (not in the dev container): starts a container from the same image as the dev container
  ([.devcontainer/Dockerfile](.devcontainer/Dockerfile)). The Docker options are currently written for
  **Windows with WSL2 + WSLg**; on native Linux you have to adapt the X11/Wayland options in its `docker run` call.

To see the live car, use the dev container and `scripts/rviz.sh`.

### Replay a recorded bag

Copy a rosbag2 (MCAP) directory into [bags/](bags), then:

```bash
scripts/view_bag.sh <name> [--rate 2] [--no-images] [--loop] [--partial] [--no-cloud]
```

The bag is replayed in an isolated DDS domain (it does not touch the car's ROS graph). RViz2 shows the robot and
TF moving, the complete recorded path, the camera images with feature tracks, a dashboard (speed, stereo skew,
tracking time) and the recorded nvblox map. It uses the same layout, [rviz/bitt.rviz](rviz/bitt.rviz), as the live view
(topics that were not recorded stay empty); the viewer switches on the nvblox and **Bag replay** groups and writes layout
changes you save in RViz back to that file. Without a bag argument the newest bag in `bags/` is used.
`--partial` replays only the complete files of a bag that was not copied completely.

## 🕹️ Simulation

`bitt_sim` runs the car in [Gazebo](https://gazebosim.org) without hardware. Everything the Jetson and ESP32 provide is
simulated (steering and drive, odometry, stereo cameras and camera info); Nav2 runs with the car's real configuration, so
the topics match the [topic reference](docs/software.md) and your own nodes and the [examples](#-examples) work unchanged.
The package is built into the dev container, so run everything below inside it.

```bash
source scripts/bitt_env.sh replay        # isolate the sim from the car (see the warning below)
ros2 launch bitt_sim sim.launch.py scenario:=parking_lot
```

| Argument | Default | |
|---|---|---|
| `scenario` | `parking_lot` | World to load, looked up in [scenarios/](scenarios) (or `$BITT_SCENARIOS_DIR`) as `<name>.sdf`, `i8_<name>.sdf` or `<name>/*.sdf` |
| `world` | | Path to a world `.sdf`; overrides `scenario` |
| `gui` | `false` | Also open the Gazebo window (needs a display, see [RViz2](#-rviz2)) |
| `foxglove` | `true` | Start the Foxglove bridge on port 8765 |

Put the world files you want to use into [scenarios/](scenarios) (its contents are git-ignored; ask your instructors for
the course scenarios). If the scenario is not found, the launch lists the worlds it did find.

To look at the sim, use [Foxglove](#-foxglove-recommended) with `ws://localhost:8765`, or `scripts/rviz.sh`.
Send goals with [examples/goal.py](examples/goal.py) or the usual Nav2 tools.

> [!WARNING]
> The sim publishes the same topics as the car (`/cmd_vel`, `/steer_cmd`, ...). Run `source scripts/bitt_env.sh replay` in
> every terminal that takes part in the sim first: it moves the terminal to an isolated DDS domain inside the container,
> so the sim and the real car cannot see each other. Switch back with `source scripts/bitt_env.sh live`.

Sim code changes under [ros_ws/src/bitt_sim/](ros_ws/src/bitt_sim) only take effect after rebuilding the dev container.

## ❓ Hidden bonus

There is a hidden bonus on the car. Can you find it, decode it and tell us what it is?
