#!/bin/bash
robotName=robot_2dw1c
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /$robotName/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/$robotName/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args -r /model/$robotName/odometry:=/$robotName/odom & 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/$robotName/cmd_vel
