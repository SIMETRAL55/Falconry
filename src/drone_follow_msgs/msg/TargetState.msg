# State of the operator-locked target, published by perception_node.
# Field spec: docs/drone_follow_design.md §3.
std_msgs/Header header
bool      target_visible       # true only if the LOCKED id is in this frame
int32     track_id             # -1 when none locked
float32   u                    # target center pixel x
float32   v                    # target center pixel y
float32   bbox_w
float32   bbox_h
float32   range_m              # depth at target center; NaN if unavailable
geometry_msgs/Point   position_cam     # 3D in camera optical frame (x right, y down, z forward); NaN if no depth
geometry_msgs/Vector3 velocity_world   # estimated target velocity (ENU); 0 until enough samples
