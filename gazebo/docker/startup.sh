#!/bin/bash
# for ssh server
sudo /etc/init.d/ssh start

# for ROS2 colcon_ws
if [ ! -f /home/ubuntu/colcon_ws/install/setup.bash ]; then
    mkdir -p /home/ubuntu/colcon_ws/src

    ln -sf \
      /home/ubuntu/git/amr/gazebo/amr_robot_launcher \
      /home/ubuntu/colcon_ws/src/amr_robot_launcher

    chown -R ubuntu:ubuntu /home/ubuntu/colcon_ws

    sudo -u ubuntu bash -c '
        source /opt/ros/jazzy/setup.bash
        cd ~/colcon_ws
        colcon build'
fi

# for gazebo-rcll
if [ ! -f /home/ubuntu/git/gazebo-rcll ]; then
    cd /home/ubuntu/git
    git clone https://github.com/wadaru/gazebo-rcll
fi

exec /entrypoint.sh
