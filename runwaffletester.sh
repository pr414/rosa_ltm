#!/usr/bin/env bash 
#tmux
pip install defusedxml
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
cd /app/catkin_ws
source /opt/ros/noetic/setup.bash

###### CREATE LAUNCH FILE FOR TEST AFTER CLONING ###########
# Define the target directory and file
TARGET_DIR="/app/catkin_ws/src/aws-robomaker-small-house-world/launch"
TARGET_FILE="$TARGET_DIR/waffle_apartment.launch"
echo "Ensuring target directory exists: $TARGET_DIR"
# Create the directory structure if it doesn't already exist
mkdir -p "$TARGET_DIR"
echo "Creating ROS launch file: $TARGET_FILE"
# Use a heredoc to write the launch file content securely and accurately
cat << 'EOF' > "$TARGET_FILE"
<launch>
  <!-- Load the small house world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find aws_robomaker_small_house_world)/worlds/small_house.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
  </include>

  <!-- Spawn the TurtleBot3 -->
  <param name="robot_description" command="$(find xacro)/xacro '$(find turtlebot3_description)/urdf/turtlebot3_$(env TURTLEBOT3_MODEL).urdf.xacro'"/>
  
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="screen"
        args="-urdf -model turtlebot3 -param robot_description -x 0 -y 0 -z 0.1" />
</launch>
EOF

if [ $? -eq 0 ]; then
    echo "Successfully created $TARGET_FILE"
    echo "Run 'chmod +x create_launch_file.sh' to make it executable."
    echo "Then run './create_launch_file.sh' to execute."
else
    echo "Error creating $TARGET_FILE"
fi
##############################################################

cd /app/catkin_ws
catkin_make
source devel/setup.bash
source /app/catkin_ws/devel/setup.bash
#roslaunch aws_robomaker_small_house_world view_small_house.launch
export TURTLEBOT3_MODEL=waffle
roslaunch aws_robomaker_small_house_world waffle_apartment.launch