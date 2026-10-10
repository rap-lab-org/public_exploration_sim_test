# ROS 2 Humble + Gazebo Classic 11, the only combination the environment supports.
FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-pcl-ros \
        ros-humble-octomap-server \
        ros-humble-joy \
        ros-humble-xacro \
        python3-colcon-common-extensions \
        curl unzip \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ws
