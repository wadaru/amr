FROM --platform="linux/x86_64" tiryoh/ros2-desktop-vnc:humble-amd64
LABEL maintainer="Wataru UEMURA<wataru@kdel.org>"

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
    add-apt-repository universe && \
    apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y openssh-server

### for ssh server
RUN mkdir /var/run/sshd && \
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config && \
    sed -i 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' /etc/pam.d/sshd && \
    head -n `expr \`wc /entrypoint.sh|awf print {$1}\` -1 ` /entrypoint.sh > /start-ssh.sh && \
    echo "sudo /etc/init.d/ssh start" >> /start-ssh.sh && \
    tail -n 1 /entrypoint.sh >> /start-ssh.sh && \
    cp /start-ssh.sh /entrypoint.sh && \
    rm /start-ssh.sh

ARG USERNAME=ryukoku
ARG PASSWORD=elec
WORKDIR /home/$USERNAME/

RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq \
                     net-tools git gcc build-essential vim screen ccache && \
    rm -rf /var/lib/apt/lists/*

