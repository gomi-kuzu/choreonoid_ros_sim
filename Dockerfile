FROM ghcr.io/ouxt-polaris/ros_handson/ros_handson:latest

RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws/src
# RUN git clone https://github.com/choreonoid/choreonoid.git
# RUN git clone https://github.com/choreonoid/choreonoid_ros.git
# RUN git clone https://github.com/choreonoid/choreonoid_ros2_mobile_robot_tutorial.git

# USER root

# 必要なライブラリのインストール
# RUN apt-get -y update
# RUN apt-get -yV upgrade
# RUN ./choreonoid/misc/script/install-requisites-ubuntu-22.04.sh

# RUN sed -i.org -e 's|ports.ubuntu.com|jp.archive.ubuntu.com|g' /etc/apt/sources.list \
#     && apt-get update && apt-get install -y \
#        x11-apps \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# RUN cd ~/ros2_ws
# RUN colcon build --symlink-install

# RUN echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
