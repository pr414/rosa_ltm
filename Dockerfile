FROM osrf/ros:noetic-desktop AS rosa-ros1
LABEL authors="Rob Royce"

ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false
ARG DEVELOPMENT=false

# Install linux packages
#RUN apt-get update && apt-get install -y \
#    ros-$(rosversion -d)-turtlesim \
#    locales \
#    xvfb \
#    python3.9 \
#    python3-pip \
#    python3.9-venv \
#    cargo \
#    rustc \
#    python3-empy \
#    mesa-utils \
#    libgl1-mesa-dri \
#    libgl1-mesa-glx \
#    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-$(rosversion -d)-turtlesim \
    ros-$(rosversion -d)-gazebo-ros \
    ros-$(rosversion -d)-gazebo-ros-pkgs \
    ros-$(rosversion -d)-gazebo-ros-control \
    ros-$(rosversion -d)-turtlebot3-gazebo \
    ros-$(rosversion -d)-turtlebot3-teleop \
    ros-$(rosversion -d)-turtlebot3-description \
    locales \
    mesa-utils \
    mesa-utils-extra \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglu1-mesa \
    libglx-mesa0 \
    xvfb \
    python3.9 \
    python3-pip \
    python3.9-venv \
    cargo \
    rustc \
    python3-empy \
    python3-defusedxml \
    tmux \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Create virtualenv -> CREATED TO SOLVE CONFLICTS WITH NATIVE APT-GET PYTHON PACKAGES
RUN python3.9 -m venv /opt/venv \
    && /opt/venv/bin/pip install --upgrade pip setuptools wheel

ENV PATH="/root/.cargo/bin:${PATH}"

# Activate venv for later commands
ENV PATH="/opt/venv/bin:$PATH"

# RUN apt-get clean && rm -rf /var/lib/apt/lists/* -> ADDED ROSPKG AND EMPY AS THEY WERE MISSING
RUN python3 -m pip install -U python-dotenv catkin_tools rospkg empy
RUN rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "alias start='catkin build && source devel/setup.bash && roslaunch waffle_agent agent.launch'" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

COPY . /app/
WORKDIR /app/

# Modify the RUN command to use ARG
RUN /bin/bash -c 'if [ "$DEVELOPMENT" = "true" ]; then \
        python3.9 -m pip install --user -e .; \
    else \
        python3.9 -m pip install -U jpl-rosa>=1.0.7; \
    fi'


# Install ROSA
#RUN /opt/venv/bin/pip install jpl-rosa>=1.0.7

# Ensure pip and Rust are available before installing ROSA
#RUN python3.9 -m pip install --upgrade pip setuptools wheel \
#    && apt-get update && apt-get install -y cargo rustc \
#    && /bin/bash -c 'if [ "$DEVELOPMENT" = "true" ]; then \
#        python3.9 -m pip install --user -e .; \
#    else \
#        python3.9 -m pip install -U jpl-rosa>=1.0.7; \
#    fi'

#CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
#    roscore > /dev/null 2>&1 & \
#    sleep 5 && \
#    if [ \"$HEADLESS\" = \"false\" ]; then \
#    rosrun turtlesim turtlesim_node & \
#    else \
#    xvfb-run -a -s \"-screen 0 1920x1080x24\" rosrun turtlesim turtlesim_node & \
#    fi && \
#    sleep 5 && \
#    echo \"Run \\'runwaffletester.sh\\' to launch the Gazebo test environment, then run \\`start\\` to build and launch the rosa_waffle_bot controller.\" && \
#    /bin/bash"]

CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    roscore > /dev/null 2>&1 & \
    sleep 5 && \
    echo \"Run runwaffletester.sh to launch the Gazebo test environment, then run \\`start\\` to build and launch the rosa_waffle_bot controller.\" && \
    /bin/bash"]



# pip install empy
# pip install rospkg
# pip install defusedxml

# roslaunch gazebo_ros empty_world.launch world_name:="$HOME/.gazebo/worlds/small_house/small_house.world" &
# roslaunch gazebo_ros empty_world.launch world_name:=/app/.gazebo/worlds/small_house/small_house.world
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/app/.gazebo/models
# export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/app/.gazebo/worlds
# export GAZEBO_TEXTURE_PATH=/app/.gazebo/models:/root/.gazebo/models


## LINK root/.gazebo to app/.gazebo
# mkdir -p /root/.gazebo
# ln -s /app/.gazebo/models /root/.gazebo/models
# ln -s /app/.gazebo/worlds /root/.gazebo/worlds

########## WORKING ##############################################################
# mkdir -p catkin_ws/src
# cd catkin_ws/src
# git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
# cd /app/catkin_ws
# source /opt/ros/noetic/setup.bash
# catkin_make
# source devel/setup.bash
# source /app/catkin_ws/devel/setup.bash
# roslaunch aws_robomaker_small_house_world view_small_house.launch
### IF WE WANT TO DIRECTLY USE GAZEBO, WE NEED TO EXPORT GAZEBO PATHS AS ABOVE

## LAUNCHING TURTLEBOT3 IN GAZEBO MAP
# export TURTLEBOT3_MODEL=waffle
# roslaunch my_robot_launch turtlebot3_small_house.launch

### CONTROLLING BOT WITH TELEOP
# IF NOT DETECTED: apt-get install -y ros-noetic-turtlebot3-teleop
## THEN
# rosrun turtlebot3_teleop turtlebot3_teleop_key

## CHECK CAMERA FEED FROM ROBOT
# source /opt/ros/noetic/setup.bash
# source /app/catkin_ws/devel/setup.bash
# rqt_image_view

###### RECORD RAW DATA TO BE PUT IN THE DB
# rosbag record -O dataset.bag /camera/rgb/image_raw /odom /clock

###############################################################################


# General Understanding Notes:
# The catkin_make command creates packages for every folder inside the src directory in the scope it is invoked.
# There is a REDUNDANCY, because we both install rosa with python_3.9 and also haved the files inside /rosa/
# We can technically define our own turtle_agent node and provide the tools to be specific to the turtlebot3_waffle functionalities, to be ran in Gazebo.