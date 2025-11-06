#!/usr/bin/env bash 
#tmux 
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
cd /app/catkin_ws
source /opt/ros/noetic/setup.bash
### EDIT LAUNCH FILE FOR GAZEBO WITH WAFFLE ROBOT ###
catkin_make
source devel/setup.bash
source /app/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch aws_robomaker_small_house_world small_apartment.launch