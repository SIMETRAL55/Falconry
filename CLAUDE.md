# Drone Detect → Select → Follow (PX4 + ROS 2 + Gazebo)

## Project context
Fork of `monemati/PX4-ROS2-Gazebo-YOLOv8`. Goal: operator selects a YOLO-tracked
object; the drone keeps it framed (gimbal) and follows it at a fixed standoff (body).
This is detection + tracking + visual servoing — **NOT SLAM**. No map is built.

The full spec lives in `docs/drone_follow_design.md`. **Treat that document as the
source of truth.** Implement it milestone by milestone (design §7). Do not jump ahead.

## Stack
- ROS 2 Humble, Python (rclpy), colcon
- PX4 SITL (PX4-Autopilot) + Micro XRCE-DDS Agent
- Gazebo (Garden/Harmonic) bridged via `ros_gz_bridge`
- YOLOv8 (ultralytics) with built-in ByteTrack (`model.track(persist=True)`)
- `px4_msgs` / `px4_ros_com` for offboard control
- New packages: `drone_follow` (nodes) and `drone_follow_msgs` (`TargetState.msg`)

## Architecture rules — DO NOT VIOLATE
- Two separate nodes: `perception_node` (CV, variable latency) and `follower_node`
  (fixed 20 Hz, deterministic). **Never let CV frame time gate the control loop.**
- `follower_node` MUST publish `OffboardControlMode` + `TrajectorySetpoint` at ≥ 20 Hz
  at ALL times, including SEARCH and LOST states, or PX4 drops offboard mode.
- Gimbal keeps the target centered; body handles standoff distance + yaw-to-face.
  Do not try to fly the whole airframe to center a bounding box.
- **PX4 is NED. ROS is ENU.** State the frame explicitly at every conversion boundary
  and leave a comment. Frame/sign errors are the #1 expected bug — be paranoid here.

## Known traps — RESPECT THESE
- The gimbal is driven over **gz-transport** topics (`/gimbal/cmd_pitch`,
  `/gimbal/cmd_yaw`), NOT ROS topics. Bridge them or publish gz-transport directly.
- The base repo bridges only the RGB `Image`. You MUST also bridge `/camera_info`
  (intrinsics) and the depth image. No intrinsics = no deprojection = no following.
- Do NOT hardcode gz topic names from memory. They vary by PX4 version. Ask the human
  to run `gz topic -l` against the live sim and confirm names before using them.
- ByteTrack reassigns track IDs after occlusion. ID-lock alone is insufficient;
  implement REACQUIRE by class + last-known position (design §6).
- Sample depth as the **median over the bbox center region**, not a single pixel.

## Commands
- Build: `cd ~/ros2_ws && colcon build --packages-select drone_follow_msgs drone_follow && source install/setup.bash`
- Launch follower stack: `ros2 launch drone_follow follow.launch.py`
- The simulation (PX4 SITL + XRCE agent + ros_gz_bridge) is launched SEPARATELY.
  See `docs/drone_follow_design.md` §3 and the repo `How_to_run.md`.

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

## Working agreement
- **You (Claude Code) cannot run the GPU sim.** When code needs to be exercised, write
  it, then STOP and ask the human to run the sim and paste back logs / errors / `ros2
  topic echo` output / rosbag findings. Iterate from real output, not assumptions.
- Implement ONE milestone at a time. Stop after each for human verification in the sim.
- Do NOT invent PID gains, standoff distances, or limits as if they were tuned. Mark
  them `# UNTUNED default — needs sim tuning` with a TODO.
- Prefer small, reviewable diffs. Use git. Commit per milestone with a clear message.
- If a topic name, message field, or API is uncertain, say so and ask — do not guess.
