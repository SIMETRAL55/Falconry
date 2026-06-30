# Environment Setup Runbook (expands M0)

Companion to `drone_follow_design.md` and `drone_follow_backlog.md`. This covers getting the base sim running before any feature work. Most of this is **build-time** — Claude Code can do it. Only the final "I can see it render and fly" check needs your eyes + GPU.

---

## 1. What "the repo" actually pulls in

`monemati/PX4-ROS2-Gazebo-YOLOv8` is a thin wrapper. The real dependency graph:

| Component | Source | Notes |
| --- | --- | --- |
| Ubuntu 22.04 | host / base image | Required for ROS 2 Humble. |
| ROS 2 Humble | apt (`ros-humble-desktop`, `ros-dev-tools`) | — |
| PX4-Autopilot | `git clone --recursive` + `Tools/setup/ubuntu.sh` + `make px4_sitl` | Tested at **v1.14.0**. Recursive submodules; long build. |
| Gazebo Garden | comes with PX4 1.14 sim | See version trap §3. |
| Micro-XRCE-DDS-Agent | eProsima, build from source | **v2.4.2** commonly matches PX4 1.14. |
| ros_gz (bridge + image) | source or Gazebo apt | Must match Gazebo version — §3. |
| px4_msgs + px4_ros_com | cloned into `~/ws_offboard_control/src`, colcon | Must match PX4 firmware — §3. |
| Python venv (`~/px4-venv`) | pip | ultralytics (YOLOv8), opencv-python, mavsdk, pygame, numpy. |
| tmuxinator | apt/gem | Orchestrates the panes. |

The repo's own contents are just the scripts (`uav_camera_det.py`, `move_car.py`, `keyboard-mavsdk-test.py`), models, worlds, and a Dockerfile that assembles all of the above.

---

## 2. Strategy: smoke-test on the prebuilt image, then build your own

Three paths. Pick deliberately.

- **(A) Prebuilt image** — `docker pull monemati/px4_ros2_gz_yolov8_image`. Fastest to a running sim. **Use it ONCE, as a throwaway smoke test** to confirm your GPU + X11 passthrough works and you can see the sim. It's a black box you can't cleanly extend.
- **(B) Build from the repo Dockerfile** — `docker build -t px4follow .`. **This is your real foundation.** You own it, can pin versions, and can add your `drone_follow` packages and extra bridges. Claude Code reads the Dockerfile, runs the build, fixes errors.
- **(C) Native on host (no Docker)** — most fragile, most "many repos," least reproducible. Avoid unless you have a reason.

**Recommended order:** A to kill the graphics-passthrough risk fast (it's the hardest thing to debug blind), then B as the foundation you build the project on. Do **not** build the project on top of (A) — you can't see inside it.

### Why this order
The two genuinely hard-to-debug risks in this setup are (1) silent version mismatches and (2) GPU/X11 passthrough. Claude Code can't see a display, so (2) is *yours* — knock it out first with the prebuilt image. Then (1) is handled by pinning versions in your own Dockerfile (§3), which the agent can build and headless-verify.

---

## 3. Version-matching checklist (the silent-failure list)

These compile clean and fail at runtime. Put every pin in your Dockerfile and in CLAUDE.md.

- [ ] **Ubuntu 22.04.** ROS 2 Humble requires it. On 24.04, stay in the container.
- [ ] **px4_msgs branch == PX4-Autopilot version.** px4_msgs `main` tracks PX4 `main`. For PX4 v1.14, use the matching `release/1.14` branch (or the tag). Mismatch → DDS topics silently don't connect.
- [ ] **ros_gz built against the right Gazebo version.** PX4 1.14 uses Gazebo **Garden**. The default `ros-humble-ros-gz` apt package is **Fortress** and conflicts with Garden. Either install ros_gz from the Gazebo (osrfoundation) apt packages for Garden, or build ros_gz from source with `export GZ_VERSION=garden` **before** `colcon build`. Wrong version → camera bridge produces nothing.
- [ ] **Micro-XRCE-DDS-Agent version** compatible with PX4's uxrce_dds_client (v2.4.2 with 1.14).
- [ ] **empy / setuptools pinned.** `colcon build` of px4_ros_com breaks with newer empy. Pin `empy==3.3.4` and a compatible setuptools.
- [ ] **NVIDIA container toolkit + flags** for YOLO GPU and Gazebo rendering: `--gpus all`, `nvidia-container-toolkit` installed, X11 via `-e DISPLAY`, mount `/tmp/.X11-unix`, run `xhost +local:` on the host.
- [ ] **Python deps in the venv:** ultralytics, opencv-python, mavsdk, pygame, numpy.

If anything in the sim "runs but does nothing," suspect this list before suspecting your code.

---

## 4. Driving Claude Code through setup

Headless = agent can verify it itself. Visual = you confirm. One prompt = one commit.

- [ ] **S0 [you]** Smoke test path (A): pull the prebuilt image, run with GPU+X11 flags, confirm the sim window renders and the car moves. *Visual.* This de-risks passthrough.
- [ ] **S1** *"Read this repo's Dockerfile and the setup runbook. List every external repo/package it installs and the exact version or branch of each. Flag anything unpinned (floating to latest) — especially px4_msgs vs PX4, and ros_gz vs Gazebo version."* — agent produces the real dependency/version map. *Headless.*
- [ ] **S2** *"Rewrite the Dockerfile to pin all versions per §3 of the runbook: PX4 v1.14, px4_msgs on the matching branch, ros_gz built with GZ_VERSION=garden, Micro-XRCE-DDS v2.4.2, empy==3.3.4. Don't change the app scripts."* — review the diff before building.
- [ ] **S3** *"Run `docker build`. When it fails, show me the error and your proposed fix before applying it."* — iterate. *Headless; the build needs no GPU.* Expect the PX4 compile to be slow.
- [ ] **S4** *"Start the container, launch the XRCE agent and PX4 SITL headless, and confirm via `ros2 topic list` and `ros2 topic echo` that the camera topic and vehicle odometry are publishing. Report what you see."* — *Headless: the agent CAN do this.* This catches the px4_msgs and ros_gz mismatches without a display.
- [ ] **S5 [you]** Full visual bring-up: run the tmuxinator session, confirm the Gazebo window shows the drone + circling car, the YOLO window shows detections, and the gimbal keys work. *Visual — this is the M0.2 baseline gate.*
- [ ] **S6** *"Run `gz topic -l` and `ros2 topic list` and record the exact names for the RGB image, depth sensor, camera_info, gimbal cmd topics, and vehicle odometry into CLAUDE.md."* — this is backlog item **0.3**, the gate for everything after.

---

## 5. Add to CLAUDE.md

```
## Environment / versions (pinned — do not float to latest)
- Ubuntu 22.04, ROS 2 Humble, Gazebo Garden
- PX4-Autopilot v1.14.0; px4_msgs on the MATCHING release branch (not main)
- ros_gz built from source with GZ_VERSION=garden (apt ros-humble-ros-gz is Fortress — wrong)
- Micro-XRCE-DDS-Agent v2.4.2; empy==3.3.4
- Build happens in Docker; the agent builds and runs HEADLESS checks (ros2 topic list/echo,
  colcon build, docker build). The agent CANNOT see the Gazebo GUI — rendering/flight
  verification is the human's job. Stop and ask for it.
- If the sim "runs but produces no data," suspect a version mismatch (px4_msgs/PX4 or
  ros_gz/Gazebo) before suspecting application code.
```
