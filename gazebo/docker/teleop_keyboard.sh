#!/bin/bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/simple_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args -r /model/simple_robot/odometry:=/odom & 
ros2 run teleop_twist_keyboard teleop_twist_keyboard
