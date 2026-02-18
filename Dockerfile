FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Madrid

# --------------------------------------------------
# 1. Locales, herramientas base, sudo
# --------------------------------------------------
RUN apt update && apt install -y \
    locales sudo curl gnupg2 lsb-release git wget build-essential \
    && locale-gen en_US en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --------------------------------------------------
# 2. Crear usuario normal con sudo y contraseña
# --------------------------------------------------
ARG USERNAME=dev
ARG PASSWORD=ros2

RUN useradd -ms /bin/bash $USERNAME \
    && echo "$USERNAME:$PASSWORD" | chpasswd \
    && usermod -aG sudo $USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# --------------------------------------------------
# 3. Instalar ROS 2 Humble + herramientas principales
# --------------------------------------------------
USER root

RUN apt update && apt install -y curl gnupg2 software-properties-common
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list'
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-tf2-tools \
    python3-argcomplete \
    python3-colcon-common-extensions

# --------------------------------------------------
# 4. MoveIt 2
# --------------------------------------------------
RUN apt install -y ros-humble-gazebo-ros
RUN apt update && apt install -y ros-humble-moveit ros-humble-moveit-planners
RUN apt install python3-vcstool
RUN apt-get update && apt-get install -y ros-humble-ros-testing
RUN apt install -y ros-humble-controller-manager-msgs
RUN apt install -y ros-humble-apex-test-tools 
RUN apt install -y ros-humble-plotjuggler-ros


RUN apt update && apt install -y \
    freeglut3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libxmu-dev \
    libxi-dev

RUN apt install -y mesa-utils
RUN apt install -y ros-humble-graph-msgs
RUN apt install -y ros-humble-rviz-visual-tools
RUN apt install -y ros-humble-control-toolbox
RUN apt install -y ros-humble-py-binding-tools
RUN apt install -y ros-humble-ur


# Install ROS 2 Development Tools:
RUN apt update && apt install -y ros-dev-tools
RUN apt install -y ros-humble-xacro

# ROS2 Control + ROS2 Controllers:
RUN apt install -y ros-humble-ros2-control
RUN apt install -y ros-humble-ros2-controllers
RUN apt install -y ros-humble-gripper-controllers

# Gazebo for ROS2 Humble:
RUN apt install -y gazebo
RUN apt install -y ros-humble-gazebo-ros2-control
RUN apt install -y ros-humble-gazebo-ros-pkgs

# xacro:
RUN apt install -y ros-humble-xacro
# Install CycloneDDS RMW for ROS 2 Humble to fix cycle time issues in humble-moveit (temporary fix):
RUN apt install -y ros-humble-rmw-cyclonedds-cpp 


# --------------------------------------------------
# 5. Navigation2
# --------------------------------------------------
RUN apt update && apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# --------------------------------------------------
# 6. SLAM Toolbox
# --------------------------------------------------
RUN apt update && apt install -y ros-humble-slam-toolbox
RUN apt install -y ros-humble-turtlebot3 
RUN apt install -y ros-humble-turtlebot3-msgs 
RUN apt install -y ros-humble-turtlebot3-bringup
RUN apt install -y ros-humble-turtlebot3*

# --------------------------------------------------
# 7. Aerostack2 (binario)
# --------------------------------------------------
# RUN apt update && apt install -y ros-humble-aerostack2


# --------------------------------------------------
# 7. Aerostack2 (source)
# --------------------------------------------------
RUN apt update
RUN apt install git python3-rosdep python3-pip python3-colcon-common-extensions -y
WORKDIR /ros2_ws/src
RUN git clone https://github.com/aerostack2/aerostack2.git
RUN rosdep init
# USER $USERNAME
USER root
RUN rosdep update
WORKDIR /ros2_ws
RUN rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
RUN apt-get update
# RUN apt upgrade
RUN apt install -y tmux
RUN apt install -y tmuxinator
RUN apt install python3-pyqt5
RUN apt install dbus-x11 -y
RUN apt install libcanberra-gtk-module libcanberra-gtk3-module -y
RUN pip install PySimpleGUI-4-foss
RUN apt install gnome-terminal -y
RUN groupadd -f video \
 && groupadd -f kvm \
 && usermod -aG video,kvm dev

WORKDIR /ros2_ws/src
RUN git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git

# IFRA-Cranfield/IFRA_ObjectPose:
RUN git clone https://github.com/IFRA-Cranfield/IFRA_ObjectPose.git

# IFRA-Cranfield/IFRA_LinkPose:
RUN git clone https://github.com/IFRA-Cranfield/IFRA_LinkPose.git

# IFRA-Cranfield/ros2_RobotiqGripper:
RUN git clone https://github.com/IFRA-Cranfield/ros2_RobotiqGripper.git


# --------------------------------------------------
# 8. Herramientas gráficas
# --------------------------------------------------
# RUN apt update && apt install -y \
#     rviz2 \
#     rqt \
#     rqt-common-plugins

COPY move_group_interface_improved.h /opt/ros/humble/include/moveit/move_group_interface/

# --------------------------------------------------
# 9. Configurar entorno ROS automáticamente
# --------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "cd /ros2_ws" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /ros2_ws
RUN echo "colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  --symlink-install --parallel-workers 3"
RUN echo "export TURTLEBOT3_MODEL=waffle"
RUN echo "export LDS_MODEL=HLS_LFCD2"
RUN echo "source install/setup.bash"
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

