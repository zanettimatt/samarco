ARG ROS_DISTRO=humble
FROM gezp/ubuntu-desktop-ros:humble-gazebo-fortress

# Instale dependências ROS 2
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \ 
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-core \
    ros-${ROS_DISTRO}-ament-package \
    ros-humble-ament-cmake-clang-format \
    ros-humble-joint-state-publisher \
    lsb-release gnupg \
    ros-humble-teleop-twist-keyboard && rm -rf /var/lib/apt/lists/*
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# Configure workspace
WORKDIR /workspace
COPY ./SMRC_Project /workspace/automation_ws
# Configure entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
# Corrigir os caracteres de fim de linha
RUN apt-get update && apt-get install -y dos2unix && dos2unix /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["bash","/entrypoint.sh"]
CMD ["bash"]
