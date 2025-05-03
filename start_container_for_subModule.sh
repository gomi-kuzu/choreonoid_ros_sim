docker run --name for_sub_wrs_env -v ./ros_src:/home/ubuntu/ros2_ws/src:z --net=host --ipc=host --privileged --device=/dev/input/js0:/dev/input/js0 ghcr.io/ouxt-polaris/ros_handson/ros_handson:latest
