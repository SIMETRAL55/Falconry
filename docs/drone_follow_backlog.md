# Drone Detect → Select → Follow — Build Backlog

Companion to `drone_follow_design.md`. **Rule: one checkbox = one Claude Code session = one git commit = one sim verification.** Do not batch. Do not start a sub-task until the one above it has passed its *Verify* gate in the running sim. Items marked **[human]** are yours, not the agent's.

---

## M0 — Baseline & environment (gates everything)

- [ ] **0.1 [human]** Fork the repo, clone, `git init` if needed. Add `docs/drone_follow_design.md` and root `CLAUDE.md`. Commit.
- [ ] **0.2 [human]** Bring up the base sim per the repo (Docker pull or manual). *Verify:* PX4 SITL, Gazebo, the circling car, and the YOLO window all start; gimbal keys (`j/k/n/m`) move the camera.
- [ ] **0.3 [human]** Inventory the live sim: `gz topic -l`, `ros2 topic list`. Record the EXACT names for RGB image, depth sensor, camera_info, `/gimbal/cmd_pitch|yaw`, and vehicle odometry. Paste them into `CLAUDE.md`. *Verify:* every name below is real, not assumed. **Nothing downstream is reliable until this is done.**
- [ ] **0.4 [human]** Confirm YOLOv8 detects the sim car. Note the confidence. *Verify:* car is detected as "car" with usable confidence. If weak, flag for M7.4 (fine-tune) — do not paper over it.

## M1 — Package scaffold & tracking

- [ ] **1.1** Scaffold `drone_follow_msgs` with `TargetState.msg` (design §3). *Verify:* `colcon build` clean; `ros2 interface show drone_follow_msgs/msg/TargetState` correct.
- [ ] **1.2** Scaffold `drone_follow` Python package: no-op `perception_node` + `follower_node` entry points, `launch/` dir, `setup.py`, `package.xml`. *Verify:* builds; both nodes run as no-ops.
- [ ] **1.3** Port the existing `uav_camera_det.py` logic into `perception_node` (subscribe RGB, YOLO predict, OpenCV display). *Verify:* window shows the same detections as the base repo.
- [ ] **1.4** Switch `.predict()` → `.track(persist=True, tracker="bytetrack.yaml")`, filter to car class, overlay track IDs. *Verify:* the circling car keeps ONE id for a full loop.
- [ ] **1.5** Log every id change for the locked-class object. *Verify:* you have a baseline count of how often ids switch (informs M6.4).

## M2 — Selection & TargetState (image-space only)

- [ ] **2.1** Maintain `{id: bbox}` for current frame; add OpenCV mouse callback; click sets `locked_id` to the track under the cursor; render the locked box distinctly. *Verify:* clicking the car locks it; clicking elsewhere doesn't.
- [ ] **2.2** Keyboard fallback: `[`/`]` cycle ids, `c` clear lock. *Verify:* works without the mouse.
- [ ] **2.3** Publish `TargetState` with image fields only (`target_visible`, `track_id`, `u`, `v`, `bbox_w/h`; range/position = NaN). *Verify:* `ros2 topic echo /target/state` tracks the car; `target_visible` flips false when it leaves frame.
- [ ] **2.4** Ensure the display loop never blocks publishing. *Verify:* topic rate stays steady when the window is busy/resized.

## M3 — 3D: intrinsics, depth, deprojection, velocity

- [ ] **3.1 [human + agent]** Add `/camera_info` to the bridge (human adds the arg with the real name from 0.3); `perception_node` subscribes, stores `K`. *Verify:* `fx, fy, cx, cy` are sane.
- [ ] **3.2 [human + agent]** Add the depth image to the bridge; subscribe and store the latest frame. *Verify:* depth received; **confirm units (m vs mm)** before using them.
- [ ] **3.3** Deproject the locked target: median depth over a bbox-center patch → `(X,Y,Z)` camera-optical; fill `range_m` + `position_cam`. *Verify:* `range_m` matches the true sim distance (read car + drone pose via `gz`) within ~10%.
- [ ] **3.4** Handle depth holes / NaNs without crashing (fallback, mark unavailable). *Verify:* graceful behavior when target is past max range or occluded.
- [ ] **3.5** Transform `position_cam` → world/ENU using gimbal angles + vehicle pose. *Verify:* computed world position ≈ the car's known world position.
- [ ] **3.6** Estimate `velocity_world` via EMA/alpha-beta finite difference on world positions. *Verify:* magnitude ≈ the car's tangential speed, direction sane — not noise.

## M4 — Gimbal-only servo (safest perception→actuation loop)

- [ ] **4.1** Add the gimbal command path (ROS→gz bridge for `/gimbal/cmd_*`, or a direct gz publisher in `follower_node`). *Verify:* open-loop — commanding a fixed angle moves the gimbal.
- [ ] **4.2** `follower_node` subscribes `TargetState`; PID on normalized vertical error → pitch, horizontal error → yaw; clamp to joint limits. **Body does not move.** *Verify:* gimbal keeps the car centered through a full loop; tune the two gimbal PIDs.
- [ ] **4.3** On `target_visible=false`, hold last gimbal command (no snapping). *Verify:* no wild gimbal motion on dropout.

## M5 — Body following (fixed altitude, chase geometry)

- [ ] **5.1** Offboard plumbing: timer publishing `OffboardControlMode` + `TrajectorySetpoint` at 20 Hz; arm + enter offboard (mirror the repo's `px4_ros_com` offboard_control). *Verify:* drone arms, holds a fixed hover setpoint in offboard, heartbeat steady.
- [ ] **5.2** Forward control: standoff PID on `(Z − d*)` → `vx_body`; altitude hold; no yaw/lateral yet. **Test on a STATIONARY target** (stop the car). *Verify:* drone advances/retreats to hold `d*` on a still target.
- [ ] **5.3** Yaw-to-face: yaw the body to wash gimbal yaw toward 0 → `yaw_rate`. *Verify:* body rotates to face the target; gimbal yaw returns toward center.
- [ ] **5.4** Lateral correction from residual horizontal error → `vy_body` (small gain). *Verify:* reduces side offset without oscillating.
- [ ] **5.5** Combine all three; re-enable the circling car; tune PIDs. *Verify:* holds ~`d*` through a full loop. Lag is expected here — that's M6.1.
- [ ] **5.6** Confirm geometry is **chase** only (design §8). *Verify:* orbit/top-down code paths are absent, not just disabled.

## M6 — Feed-forward, state machine, safety

- [ ] **6.1** Add `velocity_world` feed-forward to the setpoint. *Verify:* lag on the circle visibly drops vs 5.5.
- [ ] **6.2** State machine enum + transitions: IDLE / SEARCH / LOCK / FOLLOW / REACQUIRE / HOLD. Heartbeat continues in ALL states. *Verify:* transitions log correctly; offboard never drops across a transition.
- [ ] **6.3** SEARCH: hold position, slow gimbal/yaw sweep until a selection arrives. *Verify:* drone waits safely, no drift.
- [ ] **6.4** REACQUIRE: on `lost > N` frames, hold, point gimbal at last-known world bearing, re-detect the car class near last-known, re-lock. *Verify:* survives a deliberate occlusion / id switch and re-locks the SAME car.
- [ ] **6.5** HOLD / RTL on reacquire timeout. *Verify:* safe, predictable behavior on permanent loss.
- [ ] **6.6** Safety clamps: `v_max`, `yaw_rate_max`, accel slew, altitude floor/ceiling, geofence radius, min/max standoff. *Verify:* each limit actually triggers when you command past it.
- [ ] **6.7** Final integrated tune on the full loop. *Verify:* stable follow over multiple laps + one occlusion recovery.

## M7 — Robustness (optional, after M6 is solid)

- [ ] **7.1** Harder target path (figure-8 / variable speed) via `move_car.py`. *Verify:* follow holds.
- [ ] **7.2** Multiple cars in the scene. *Verify:* selection and REACQUIRE pick the intended one, not the nearest.
- [ ] **7.3** Other target classes (person, truck). *Verify:* class filter + lock generalize.
- [ ] **7.4** If 0.4 flagged weak detection: fine-tune YOLO on sim frames. *Verify:* confidence improves on the sim car.
- [ ] **7.5** Record rosbags; quantify standoff RMSE and centering error per run. *Verify:* you have numbers, not vibes.
- [ ] **7.6** Headless smoke test of both nodes (no GUI) for CI. *Verify:* nodes start and exchange topics without a display.

---

### Sequencing rules
- **0.3 before anything that names a topic.** Wrong names = silent failures.
- **M3 before M4/M5** — no range, no following.
- **M4 fully solid before M5** — debug perception→actuation on the cheap actuator first.
- **5.2 on a stationary target before a moving one** — isolate standoff control from target motion.
- **No invented gains.** Every PID value enters as an untuned default with a TODO and gets tuned at its own *Verify* gate.
