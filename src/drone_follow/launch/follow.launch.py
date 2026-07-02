"""Launch the follower stack: perception_node + follower_node.

The simulation (PX4 SITL + XRCE agent + ros_gz_bridge) is launched SEPARATELY
— see How_to_run.md. This launch can optionally start an EXTRA parameter_bridge
for the topics the base repo does not bridge (camera_info, depth, gimbal), via
start_extra_bridge:=true.

!! The gz-side topic names below are the design-doc defaults (§3) and VARY BY
!! PX4 VERSION. Confirm against `gz topic -l` on the live sim before trusting
!! them, and override with launch arguments if they differ.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# CONFIRMED against the live sim (`gz topic -l`, PX4 v1.14 + OakD-Lite model):
# sensors publish on plain names, not /world/... scoped paths.
GZ_CAMERA_INFO = '/camera_info'
GZ_DEPTH = '/depth_camera'


def generate_launch_description():
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    start_extra_bridge = LaunchConfiguration('start_extra_bridge')
    auto_arm = LaunchConfiguration('auto_arm')

    return LaunchDescription([
        DeclareLaunchArgument('rgb_topic', default_value='/camera'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_camera'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info'),
        DeclareLaunchArgument('start_extra_bridge', default_value='false',
                              description='Also bridge camera_info/depth/gimbal '
                                          '(verify gz names first!)'),
        DeclareLaunchArgument('auto_arm', default_value='false',
                              description='Follower switches PX4 to offboard '
                                          'and arms (SITL testing only)'),

        Node(
            package='drone_follow',
            executable='perception_node',
            output='screen',
            parameters=[{
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
            }],
        ),
        Node(
            package='drone_follow',
            executable='follower_node',
            output='screen',
            parameters=[{'auto_arm': auto_arm}],
        ),

        # Extra bridge for what the base repo does not bridge (design §3):
        # camera_info + depth (gz->ROS), gimbal commands (ROS->gz).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            condition=IfCondition(start_extra_bridge),
            arguments=[
                f'{GZ_CAMERA_INFO}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'{GZ_DEPTH}@sensor_msgs/msg/Image[gz.msgs.Image',
                '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
                '/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double',
            ],
        ),
    ])
