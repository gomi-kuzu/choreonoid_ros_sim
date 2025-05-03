cd ~/ros2_ws
apt update && apt -y  upgrade 
rosdep install -y --from-paths ~/ros2_ws/src --ignore-src
colcon build --symlink-install --packages-select my_mobile_robot_sub
echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
