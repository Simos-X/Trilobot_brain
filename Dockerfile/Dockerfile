FROM ros:humble

RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

RUN apt-get update \
    && apt-get install -y ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2-controllers-test-nodes \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-twist-mux \
    ros-humble-imu-tools \
    ros-humble-image-transport-plugins \
    ros-humble-ffmpeg-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-depthai-ros \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-teleop-twist-keyboard \
    ros-humble-topic-tools 


ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp 
ENV ZENOH_ROUTER_CHECK_ATTEMPTS=-1
ENV ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:0"];scouting/multicast/enabled=true'

ARG USER_ID=ros2_user

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ARG USER_ID
USER $USER_ID

CMD ["bash"]
