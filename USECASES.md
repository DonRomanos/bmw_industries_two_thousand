<h1 align="center">🎯 BITT Use Cases</h1>

<p align="center"><em>Four missions. One car. Your algorithms.</em></p>

<p align="center"><img src="docs/kitt-scanner.svg" alt="Animated red KITT-style scanner light" width="400"></p>

<p align="center">
  <img alt="Use cases" src="https://img.shields.io/badge/use%20cases-4-E7352C">
  <img alt="Runs on" src="https://img.shields.io/badge/runs%20on-your%20laptop-0A7BBB">
  <img alt="Link" src="https://img.shields.io/badge/link-Wi--Fi%20only-22314E">
  <img alt="Driving" src="https://img.shields.io/badge/driving-Nav2-76B900">
</p>

<p align="center">
  <a href="#-how-you-work-with-the-car">How you work</a> ·
  <a href="#-1-intent-detection">1 Intent</a> ·
  <a href="#-2-parking">2 Parking</a> ·
  <a href="#-3-memory-and-monitoring">3 Memory</a> ·
  <a href="#-4-your-own-use-case">4 Your own</a> ·
  <a href="README.md">README</a>
</p>

---

Four tasks to tackle with BITT. Most of them have a scenario in the [simulation](README.md#-simulation), so you can
develop without the car and then show it on the real one; not every use case has exactly a matching scenario. The
scenario worlds will be added to [scenarios/](scenarios) (the folder's contents are git-ignored; ask your instructors
for the files).

| | Use case | In one sentence |
|:-:|----------|-----------------|
| 🕵️ | [**1 · Intent detection**](#-1-intent-detection) | Watch people, work out what they want and react: follow, flee or warn |
| 🅿️ | [**2 · Parking**](#-2-parking) | Find a spot and park where the user says: "behind that box", "next to the yellow car" |
| 🧠 | [**3 · Memory and monitoring**](#-3-memory-and-monitoring) | Answer questions about what the car saw and did |
| 💡 | [**4 · Your own idea**](#-4-your-own-use-case) | Build something cool with the car and show us |

## 📡 How you work with the car

```text
 ┌────────────────────────┐   Wi-Fi (ROS 2)   ┌────────────────────┐
 │  Your laptop           │ ◀──────────────── │  BITT              │
 │  dev container         │   camera, map,    │  sensors, SLAM,    │
 │  YOUR ALGORITHMS       │   odometry        │  Nav2              │
 │                        │ ────────────────▶ │                    │
 └────────────────────────┘   goals, lights,  └────────────────────┘
                              horn
```

**Your algorithms run on your own machine** (the dev container on your laptop). The car is reached **only through
Wi-Fi**, over ROS 2: you subscribe to its topics for sensing and publish to its topics, send Nav2 goals or call its
services to act. Nothing of yours runs on the Jetson. The simulation works the same way, so your code runs unchanged
against the sim and the real car. Your code belongs in [student_ws/src](student_ws/src).

<details open>
<summary><b>What the car offers</b> (details in the <a href="docs/software.md">topic reference</a> and the <a href="README.md#-examples">examples</a>)</summary>

| Capability | Where |
|------------|-------|
| 📷 Left / right camera pictures (JPEG, low rate) | `/preview/left/compressed`, `/preview/right/compressed`, see [examples/camera.py](examples/camera.py) |
| 🔙 Rear camera (only if connected) | `/back_camera/image/compressed` |
| 🚧 Obstacles around the car | `/local_costmap/costmap`, `/nav_debug/lethal_local`, see [examples/obstacles.py](examples/obstacles.py) |
| 📍 Where the car is and where it has been | `/tf` (`odom` → `base_link`), `/visual_slam/tracking/odometry`, `/visual_slam/tracking/slam_path` |
| 🧭 Drive to a point | Nav2 goals: `/goal_pose` or the `navigate_to_pose` action, see [examples/goal.py](examples/goal.py); cancel with the *Cancel navigation* button or `scripts/cancel_navigation.py` |
| 💬 Say something | Lights, indicators, hazard and horn: [examples/lights.py](examples/lights.py) |
| 🎛️ Steering and speed as commanded | `/steer_cmd`, `/cmd_vel`, `/steer_state` (read them; see the warning below before publishing to the first two) |

</details>

> [!IMPORTANT]
> **Drive with Nav2.** Publishing to the raw `/steer_cmd` and `/cmd_vel` goes straight to the motor and steering and
> bypasses Nav2's obstacle avoidance. Use it only in exceptions, and carefully: slow, short, with the remote controller
> in your hand. Nav2 is also what keeps the car safe.

> [!WARNING]
> Keep the remote controller within reach whenever the car can move. The remote always wins.

Things to keep in mind:

- 🚗 The car has **Ackermann steering**: it cannot turn on the spot, and goals behind or beside it can be rejected.
- 🔋 `/wheel_odom` and `/batt_status` are **not published yet**. For speed and distance use the SLAM odometry.
- 📶 Wi-Fi limits what you can receive: use the `/preview/...` topics and the costmaps. The raw and depth images are far
  too large (see "Topics visible but no data" in [TROUBLESHOOTING.md](TROUBLESHOOTING.md)).
- 🔇 The car has no speaker or screen: its way of talking is lights, indicators, hazard and the horn.
- 🕹️ Test in the sim first (`source scripts/bitt_env.sh replay`), then on the car (`source scripts/bitt_env.sh live`).

---

## 🕵️ 1. Intent detection

> **The car watches its surroundings, works out what the people around it intend and reacts appropriately.**

| Who | What the car does |
|-----|-------------------|
| 🙂 Someone **friendly** | Follows them |
| 🔪 Someone **threatening** | Flees and warns. In the simulation scenario, a person draws a knife and points it at the car |
| 🧒 A **child in front of the car** | Warns. It warns for more than a child, for any person in its way |

> [!TIP]
> **Done when:** in the scenario the car shows a different, fitting reaction to each kind of person, without anyone
> touching it, and you can explain why it decided so.

## 🅿️ 2. Parking

> **The car finds a parking spot and parks where the user asks.**

Examples of requests:

- 💬 *"Park behind that box."*
- 💬 *"Park next to the yellow car."*

> [!TIP]
> **Done when:** given different requests in the parking scenario, the car ends up in the right place each time and does
> not touch anything.

## 🧠 3. Memory and monitoring

> **The car monitors its environment and can tell you about its history.**

Examples of questions:

- 💬 *"What colour was the shirt of the person I just passed?"*
- 💬 *"How long have we been driving?"*
- 💬 *"How fast was I going five minutes ago?"*

> [!NOTE]
> Be careful with time: the car's clock is only right after `scripts/set_car_time.py`, so measure durations with your own
> timestamps and not with the car's wall clock (see "Clock skew" in [TROUBLESHOOTING.md](TROUBLESHOOTING.md)).

> [!TIP]
> **Done when:** after a drive through the scenario, the car answers questions about what it saw and did correctly, and
> says so when it does not know.

## 💡 4. Your own use case

> **Have an idea? Build it and show us.**

You have a car that sees, maps, localises itself, drives to goals and talks with lights and the horn. If you have a cool
idea, try implementing it and show us.
