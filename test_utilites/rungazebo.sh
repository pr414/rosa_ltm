#!/usr/bin/env bash 
#tmux 
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
cd /app/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
source /app/catkin_ws/devel/setup.bash
#roslaunch aws_robomaker_small_house_world view_small_house.launch
export TURTLEBOT3_MODEL=waffle
roslaunch my_robot_launch turtlebot3_small_house.launch