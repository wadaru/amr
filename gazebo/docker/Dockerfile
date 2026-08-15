# FROM --platform="linux/x86_64" tiryoh/ros2-desktop-vnc:humble-amd64
# FROM --platform="linux/x86_64" tiryoh/ros2-desktop-vnc:jazzy
FROM tiryoh/ros2-desktop-vnc:jazzy
# FROM tiryoh/ros2-desktop-vnc:humble
LABEL maintainer="Wataru UEMURA<wataru@kdel.org>"

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
    add-apt-repository universe && \
    apt-get update -q && \
    apt-get upgrade -yq --allow-downgrades && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y openssh-server && \
    rm -rf /var/lib/apt/lists/*

##### for ssh server
##RUN mkdir /var/run/sshd && \
##    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config && \
##    sed -i 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' /etc/pam.d/sshd && \
##    head -n `expr \`wc /entrypoint.sh|awf print {$1}\` -1 ` /entrypoint.sh > /start-ssh.sh && \
##    echo "sudo /etc/init.d/ssh start" >> /start-ssh.sh && \
##    tail -n 1 /entrypoint.sh >> /start-ssh.sh && \
##    cp /start-ssh.sh /entrypoint.sh && \
##    rm /start-ssh.sh
COPY startup.sh /startup.sh
RUN chmod +x /startup.sh && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu-nopasswd && \
    chmod 0440 /etc/sudoers.d/ubuntu-nopasswd
ENTRYPOINT ["/startup.sh"]

# ARG USERNAME=ryukoku
# ARG PASSWORD=elec
# WORKDIR /home/$USERNAME/

RUN apt-get update -q && \
    apt-get upgrade -yq  --allow-downgrades && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
                     net-tools git gcc build-essential vim screen ccache && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update -q && \
    apt-get upgrade -yq  --allow-downgrades && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
      python3-pip python3-usb && \
    rm -rf /var/lib/apt/lists/*

# RUN apt-get update -q && \
#     apt-get upgrade -yq && \
#     DEBIAN_FRONTEND=noninteractive apt-get install -yq \
#                      ros2-humble-nav2-common && \
#     rm -rf /var/lib/apt/lists/*

### for colcon_ws
## RUN cd /home/ubuntu/colcon_ws && \
##     colcon build 
# please add the script to ~/desktop/amr-docker-setup.desktop
# because ~/colcon_ws is linked to the host machine.

### for gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update -y && \
    apt-get install -yq gz-harmonic && \
    apt-get install -yq ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers && \
    apt-get install -yq ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ros-gz-sim

### make git directory and clone the repository of our team
RUN mkdir -p /home/ubuntu/git && \ 
    cd /home/ubuntu/git && \
    git clone https://github.com/BabyTigers-R/rcll.git && \
    cd rcll && \
    apt-get update && \
    echo "build"

### make AMR environment from git
RUN mkdir -p /home/ubuntu/git && \
    cd /home/ubuntu/git && \
    git clone https://github.com/wadaru/amr && \
    cd amr && \
    # git checkout 18-prepare-simple-world && \
    mkdir -p /home/ubuntu/Desktop && \
    ln -s /home/ubuntu/git/amr/gazebo/docker/desktop/amr-gazebo-launch.desktop /home/ubuntu/Desktop/ && \
    ln -s /home/ubuntu/git/amr/gazebo/docker/desktop/amr-gazebo-launch_slam.desktop /home/ubuntu/Desktop/ && \
    echo "source ~/colcon_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "amr..."

RUN mkdir -p /home/ubuntu/git && \
    cd /home/ubuntu/git && \
    git clone https://github.com/wadaru/gazebo-rcll && \
    cd /home/ubuntu/git/amr && \
    git pull --ff-only && \
    cd /home/ubuntu/git/gazebo-rcll/ && \
    echo "gazebo-rcll."

### make myAGV 
RUN mkdir -p /home/ubuntu/git && \
    cd /home/ubuntu/git && \
    git clone https://github.com/elephantrobotics/myagv_ros2 && \
    cd myagv_ros2 && \
    git apply  /home/ubuntu/git/amr/gazebo/docker/myAGV_wataru.patch && \
    echo "myAGV."

### install  SLAM-toolbox
RUN apt-get update -q && \
    apt-get upgrade -yq  --allow-downgrades && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 \ 
                                                       ros-${ROS_DISTRO}-topic-tools ros-${ROS_DISTRO}-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*
