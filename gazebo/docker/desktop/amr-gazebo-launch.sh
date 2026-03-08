#!/bin/bash
source ~/.bashrc
source ~/colcon_ws/install/setup.bash
cd ~/git/gazebo-rcll/models
ros2 launch amr_robot_launcher sim_launch.py
