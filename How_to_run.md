
# Build

docker rm -f px4_ros2_gazebo_yolov8_container 2>/dev/null
docker build -t px4_ros2_gazebo_yolov8_image .



# Run it 

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
    --rm --name px4_ros2_gazebo_yolov8_container \
    px4_ros2_gazebo_yolov8_image


# Follower stack (drone_follow)

Inside the running container (or any shell with ROS 2 Humble + the
ws_sensor_combined px4_msgs workspace sourced):

    # build the new packages
    mkdir -p ~/ros2_ws/src && cp -r /path/to/repo/src/* ~/ros2_ws/src/
    cd ~/ros2_ws && colcon build --packages-select drone_follow_msgs drone_follow
    source install/setup.bash

    # 1) verify gz topic names FIRST (they vary by PX4 version):
    gz topic -l | grep -E 'camera_info|depth|gimbal'

    # 2) launch (bridges camera_info/depth/gimbal too if names match design §3):
    ros2 launch drone_follow follow.launch.py start_extra_bridge:=true

    # override topic names if the sim uses different ones:
    #   ros2 launch drone_follow follow.launch.py depth_topic:=/your/depth ...

Click the car in the perception window to lock it. Keys: [ ] cycle ids,
c clears the lock. Use auto_arm:=true ONLY in SITL to let the follower
switch PX4 into offboard and arm it after ~1 s of setpoint streaming.
