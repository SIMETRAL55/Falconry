# Falconry — First-Time Setup & Run Guide

Detect → select → follow: click a YOLO-detected object in the drone's
camera feed and it gimbal-tracks and chases the target in PX4 SITL +
Gazebo. This guide gets you from a fresh clone to a flying, following
drone.

## 1. Prerequisites

- Linux host with an NVIDIA GPU + driver
- [Docker](https://docs.docker.com/engine/install/) with
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  (`docker run --gpus all ...` must work)
- X11 display (a normal desktop Linux session; this has not been tested
  under Wayland-only setups)
- ~15 GB free disk space (the image is large — PX4, Gazebo, ROS 2, and
  CUDA-enabled PyTorch all live inside it)

Everything else (ROS 2 Humble, PX4-Autopilot, Gazebo, Micro-XRCE-DDS,
YOLOv8) is built into the Docker image — you do not need to install any
of it on the host.

## 2. Clone

```bash
git clone git@github.com:SIMETRAL55/Falconry.git
cd Falconry
```

## 3. Build the image

```bash
docker build -t px4follow:m0 .
```

This takes a while the first time (PX4 + Gazebo + ROS 2 + the follower
packages all compile from source). Grab a coffee.

> **Use the `px4follow:m0` tag, not `px4follow:latest`.** An earlier
> build has a gimbal-mounting bug in the generated SDF that prevents the
> drone from spawning at all. Always build fresh with the tag above.

## 4. Launch the simulator

```bash
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run --privileged -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --network=host --ipc=host --shm-size=2gb \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --rm --name falconry \
  px4follow:m0
```

This opens a tmux session (via tmuxinator) with PX4 SITL, Gazebo, the
Micro-XRCE-DDS agent, and the base YOLO detection window all running.
Give it 20-60 seconds — PX4 needs to fully boot before anything useful
happens.

**If the drone never appears / the sim seems stuck:** PX4's connection
to the Micro-XRCE-DDS agent occasionally loses the race on first start.
Check the PX4 pane for `successfully created rt/fmu/out/vehicle_odometry
data writer` — if you don't see it after ~20s, kill and restart just the
PX4 process (`Ctrl+C` in its tmux pane, re-run the `px4_sitl_default/bin/px4`
command shown in `px4_ros2_gazebo.yml`).

## 5. Build the follower stack

Open a new shell into the running container:

```bash
docker exec -it falconry bash
```

Then, inside the container:

```bash
mkdir -p ~/ros2_ws/src
cp -r /root/PX4-ROS2-Gazebo-YOLOv8/src/* ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ws_sensor_combined/install/setup.bash
colcon build --packages-select drone_follow_msgs drone_follow
source install/setup.bash
```

## 6. Confirm the Gazebo topic names

The gz-transport topic names for the camera/depth/gimbal sensors can
vary by PX4/Gazebo version. Verify before launching:

```bash
gz topic -l | grep -E 'camera_info|depth|gimbal'
```

You should see something like `/camera_info`, `/depth_camera`,
`/gimbal/cmd_pitch`, `/gimbal/cmd_yaw`. If the names differ, pass the
correct ones as launch arguments in the next step (see `--help` via
`ros2 launch drone_follow follow.launch.py -s`).

## 7. Launch the follower

```bash
ros2 launch drone_follow follow.launch.py start_extra_bridge:=true
```

This starts both `perception_node` (YOLOv8 + ByteTrack detection window)
and `follower_node` (20 Hz PX4 offboard controller + gimbal servo), and
bridges the extra gz topics the base image doesn't wire up by default.

To have the follower automatically switch PX4 into offboard mode and arm
(SITL only — never do this on real hardware without understanding the
safety implications):

```bash
ros2 launch drone_follow follow.launch.py start_extra_bridge:=true auto_arm:=true
```

**If the drone disarms/lands unexpectedly during a test:** the default
simulated battery drains fast enough to trip PX4's low-battery failsafe
during a longer session. For testing, raise the threshold and disable
the failsafe action:

```bash
px4-param set SIM_BAT_MIN_PCT 80
px4-param set COM_LOW_BAT_ACT 0
```

## 8. Fly it

1. A window titled **"drone_follow: perception"** shows the live camera
   feed with detected objects boxed and labeled (`id:N`).
2. **Click on a box** to lock that target — the box turns red and the
   status line reads `locked: N`.
3. Keyboard fallback: `[` / `]` cycle through visible track IDs, `c`
   clears the current lock.
4. Once locked and visible, the gimbal centers the target and (if armed)
   the drone flies to hold a standoff distance behind/above it.
5. If the target is lost (occluded, out of frame), the system enters
   `REACQUIRE` and tries to re-match it by object class + last-known
   position, falling back to `HOLD` after a timeout if it can't.

## Known limitations to expect

- **All control gains are UNTUNED defaults** (standoff distance, PID
  terms, search sweep amplitude/period) — marked as such in
  `follower_node.py`. Expect to tune them once you can observe real
  flight behavior.
- **YOLOv8's stock weights are angle-sensitive.** Detection is reliable
  at oblique/standoff-distance viewing angles but can misclassify
  vehicles at steep near-vertical framings (confirmed: a truck viewed
  from directly above was misclassified as a suitcase). Keep the drone
  at a reasonable standoff distance rather than hovering close and
  looking straight down. Fine-tuning YOLO on a few captured sim frames
  would remove this constraint — see `docs/drone_follow_design.md` §9.

## Where to go next

- `docs/drone_follow_design.md` — full system design and milestone plan
- `CLAUDE.md` — architecture rules, known traps, and the frame-convention
  gotchas (PX4 is NED, ROS is ENU — see this before touching any
  coordinate math)
- `How_to_run.md` — the condensed command reference this guide expands on
