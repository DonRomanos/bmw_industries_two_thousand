# Troubleshooting

Start with the doctor. Inside the dev container, with the car switched on and your laptop on the `BITT` Wi-Fi:

```bash
scripts/doctor.py
```

It checks your ROS environment, the route and ping to the car, ROS discovery, whether the important topics deliver
data, the clock difference to the car and the Nav2 action server. Every failing line says what to look at; the
sections below explain the fixes. It prints `[ ok ]`, `[warn]` (something optional is missing) or `[FAIL]` (nothing
will work until this is fixed).

| Section | Symptom |
|---------|---------|
| [Cannot ping the car](#cannot-ping-the-car) | `doctor.py`: no route / no ping answer |
| [Ping works but no topics](#ping-works-but-no-topics) | `ros2 topic list` shows only `/parameter_events` and `/rosout` |
| [Topics visible but no data](#topics-visible-but-no-data) | Topics are listed, `echo` / RViz show nothing |
| [Clock skew](#clock-skew) | Goals fail with transform errors, RViz shows "extrapolation" or "message too old" |
| [The car does not move](#the-car-does-not-move) | Goals are accepted but nothing happens |
| [RViz](#rviz) | No window, "No Image", empty maps |
| [Bag replay](#bag-replay) | Replay is silent or mixes with the car |
| [Dev container](#dev-container) | Build fails, workspace not sourced |

## Cannot ping the car

- Connect to the Wi-Fi `BITT` (ask your instructors for the password, see the [README](README.md#-connecting-to-the-car)). The car has no internet,
  so your OS may warn about it or switch back to another network: tell it to stay connected.
- Your laptop must get an address `10.42.0.x`: `ip addr` (Windows: `ipconfig`). No address means DHCP on the car
  has not answered; wait a minute after the car booted.
- The car is `10.42.0.1`. If you changed it, set `BITT_IP` on your host before opening the container.
- The car boots its own access point after a minute or so. No `BITT` network at all means the Jetson is off or
  still booting, or it is connected to another Wi-Fi: ask your instructors.
- A VPN or proxy client can capture `10.42.0.0/24`; disconnect it while working with the car.
- Windows: a "public network" profile can block ping; test again with the profile set to private.

## Ping works but no topics

`doctor.py` prints `no ROS topics visible`. The container reaches the car, but ROS discovery does not.

1. **Is the car running the pipeline?** The topics only exist while the nav/SLAM pipeline runs on the Jetson
   (`kitt-pipeline` service). Ask your instructors, or check `ros2 topic list` again after a minute.
2. **Same domain?** The car uses domain `0`; check `echo $ROS_DOMAIN_ID`. If you set `ROS_DOMAIN_ID` on your host
   it is passed into the container.
3. **Still in replay mode?** A terminal that ran `source scripts/bitt_env.sh replay` only sees the bag player.
   Run `source scripts/bitt_env.sh live` or open a new terminal.
4. **Host networking.** The container needs `--network=host` (already set in `.devcontainer/devcontainer.json`).
   It works on Linux and on Windows with Docker running inside WSL2. Docker Desktop and the default WSL2 NAT mode do
   not work; for WSL2 use mirrored networking in `%UserProfile%\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   then `wsl --shutdown` and reopen the container. `doctor.py` warns if traffic to the car leaves through a Docker
   network instead of the Wi-Fi.
5. **Windows firewall (WSL2 mirrored mode).** The Hyper-V firewall blocks incoming traffic from the car by default.
   In an administrator PowerShell:
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```
6. **Static peer.** `echo $ROS_STATIC_PEERS` must show the car's address (`10.42.0.1`). Multicast discovery is
   unreliable on Wi-Fi, so the container uses unicast to the car.
7. Restart the ROS daemon, which can cache a stale graph: `ros2 daemon stop`, then `ros2 topic list`.

## Topics visible but no data

`doctor.py` prints `advertised but no data received`: discovery works, the data does not arrive.

- **Incoming UDP is blocked** on your laptop (firewall, see the Hyper-V rule above; corporate security software).
  Discovery needs only one direction, data needs the other.
- **Too much data for Wi-Fi.** Raw images (`/visual_slam/image_*`, `/left/image_rect`), point clouds and the nvblox
  mesh are tens of MB/s and will stall. Use `/preview/left/compressed` and the costmaps. If you must look at big
  topics, subscribe with best-effort QoS (`ros2 topic echo --qos-reliability best_effort ...`).
- **QoS mismatch.** A reliable subscriber never receives from a best-effort publisher. Subscribe best-effort
  (`qos_profile_sensor_data`) when in doubt; see [examples/camera.py](examples/camera.py).
- Weak Wi-Fi: `doctor.py` shows ping loss and latency. Move closer to the car.

## Clock skew

The Jetson has no battery-backed clock and boots in 1970 until someone sets the time. If your laptop and the car
disagree by more than a few seconds, transforms and goals fail: Nav2 and RViz look up transforms at the time stamps
of the messages. `doctor.py` reports `clock difference to the car`.

- Symptoms: `Lookup would require extrapolation`, `Message removed because it is too old`, goals from the RViz
  2D Goal Pose tool are rejected or abort at once.
- Fix: the instructors set the car's time (`scripts/sync_time.sh` in the main project) before a session. Check your
  own laptop clock too (automatic time on, correct timezone), since the comparison is against your laptop.
- `examples/goal.py` sends goals without a time stamp and works with a skewed clock; the RViz goal tool does not.

## The car does not move

- You do not set the drive mode: the car's nav pipeline switches to autonomy and sends the heartbeat while Nav2 is
  commanding. Without a fresh `/jetson_heartbeat` (`scripts/doctor.py` shows it) the car does not drive; if it is
  missing, the pipeline is not running in `nav` mode: ask your instructors.
- **The remote controller always wins.** Any stick input takes over for a second. Check
  `ros2 topic echo /controller_state`.
- `doctor.py` warns `/steer_state not published` when the ESP32 is not connected to the micro-ROS agent
  (UDP 8888): no ESP means no lights, steering or driving.
- Nav2 may reject goals that are too close, behind the car or sideways: the car has Ackermann steering and does not
  turn on the spot. Try a goal 2 m ahead: `python3 examples/goal.py 2.0 0.0`.

## RViz

- **No window:** `echo $DISPLAY` must not be empty in the container. On Windows (WSLg) it works out of the box; on
  Linux run `xhost +local:` on the host. Start VS Code from a terminal where `echo $DISPLAY` shows a value.
- **"No Image":** the preview is a compressed topic; the display must use `/preview/left/compressed`. Without
  the car, replay a bag that contains it (see below).
- **Maps and costmaps are empty:** they need the nav pipeline (`doctor.py`: `/local_costmap/costmap`), and the
  fixed frame `odom` must exist: `ros2 run tf2_ros tf2_echo odom base_link`.
- RViz logs a GLSL shader error for map displays with software rendering. The maps still draw; ignore it.
- RViz is slow: rendering is in software. Disable displays you do not need in the Displays panel.

## Bag replay

- `scripts/play_bag.sh` publishes in its own isolated domain with shared memory only. Terminals you want to see
  it in need `source scripts/bitt_env.sh replay`; go back with `source scripts/bitt_env.sh live`.
- `doctor.py` in a replay terminal checks the replayed topics and skips the network and clock checks.
- Only topics that were recorded exist. Check with `python3 examples/bag_reader.py info bags/<name>`.
- Playing the raw stereo images needs a lot of CPU and memory: pass `--topics` to play only what you need.

## Dev container

- The first build needs internet (apt and the ROS base image). Later starts work offline.
- Packages you install with `sudo apt-get install` are gone when the container is rebuilt; put what you need in
  your own Dockerfile layer instead.
- `ros2` or `car_interfaces` not found: open a new terminal (the workspaces are sourced from `~/.bashrc`) or run
  `source /opt/ros/lyrical/setup.bash && source /opt/bitt_ws/install/setup.bash`. After building your own
  workspace, `source student_ws/install/setup.bash`.
