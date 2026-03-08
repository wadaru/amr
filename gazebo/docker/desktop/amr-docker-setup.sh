#!/bin/bash
source ~/.bashrc
cd ~/colcon_ws
mkdir -p src
ln -s /home/ubuntu/git/amr/gazebo/amr_robot_launcher /home/ubuntu/colcon_ws/src
colcon build
