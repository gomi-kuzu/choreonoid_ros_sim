apt update && apt -y  upgrade 
cd ~/ros2_ws/src/
git clone https://github.com/choreonoid/choreonoid.git
git clone https://github.com/choreonoid/choreonoid_ros.git
git clone https://github.com/choreonoid/choreonoid_ros2_mobile_robot_tutorial.git
./choreonoid/misc/script/install-requisites-ubuntu-22.04.sh
cd ~/ros2_ws
colcon build --symlink-install
echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
