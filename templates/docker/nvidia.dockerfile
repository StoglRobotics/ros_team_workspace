FROM nvidia/cuda:11.7.1-base-UBUNTU_DUMMY_VERSION

#Arguments maybe add defaults?
ARG user
ARG uid
ARG gid
ARG home

# make bash default
SHELL ["/bin/bash", "-c"]

# install locales
RUN apt update -y && apt install -y locales

# Configure user env
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Set the locale to include UTF-8. UTF-8 compatible locales are needed for install
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Make sure UTF-8 is supported
RUN locale

# install nala and upgrade
RUN apt update -y && apt install -y nala
RUN nala upgrade -y

# Nvidia variables
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install basic utilities
RUN nala update && nala install -y git git-lfs nano sudo tmux tree vim iputils-ping wget bash-completion pip trash-cli
RUN pip install pre-commit

# install ROS:ROS_DUMMY_VERSION dependencies
RUN nala install -y curl gnupg gnupg2 lsb-release software-properties-common && apt-add-repository universe

# Add key to keyring
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS:ROS_DUMMY_VERSION and things needed for ros development, DEBIAN_FRONTEND is needed to ignore interactive keyboard layout setting while install
RUN nala update && DEBIAN_FRONTEND=noniteractive nala install -y ros-ROS_DUMMY_VERSION-desktop python3-colcon-common-extensions python3-vcstool
RUN pip install -U rosdep && \
    rosdep init

# setup ros_team_ws
RUN git clone -b ROS_TEAM_WS_DUMMY_BRANCH https://github.com/StoglRobotics/ros_team_workspace.git /opt/RosTeamWS/ros_ws_ROS_DUMMY_VERSION/src/ros_team_workspace/
RUN cd /opt/RosTeamWS/ros_ws_ROS_DUMMY_VERSION/src/ros_team_workspace/rtwcli && pip3 install -r requirements.txt && cd -

# setup standard .bashrc
COPY bashrc ${home}/.bashrc
COPY ros_team_ws_rc_docker ${home}/.ros_team_ws_rc

# clone user into docker image, add to sudo users needed for xsharing
# the passwd delete is needed to update /etc/shadow otherwise user cannot use sudo
RUN mkdir -p ${home} && \
  echo "${user}:x:${uid}:${gid}:${user},,,:${home}:/bin/bash" >> /etc/passwd && \
  echo "${user}:x:${gid}:" >> /etc/group && \
  echo "${user} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${user}" && \
  chmod 0440 "/etc/sudoers.d/${user}" && \
  usermod -aG sudo ${user} && \
  passwd -d ${user} && \
  chown ${uid}:${gid} -R ${home}

#switch to user
USER ${user}
WORKDIR ${home}
