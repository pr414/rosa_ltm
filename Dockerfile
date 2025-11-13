# ============================================================
#  Base image: ROS Noetic (Python 3.9) → ROSA environment
# ============================================================
FROM osrf/ros:noetic-desktop AS rosa-ros1
LABEL authors="Paolo Riva"

ENV DEBIAN_FRONTEND=noninteractive
ENV HEADLESS=false
ARG DEVELOPMENT=false

# ------------------------------------------------------------
# System packages + both Python versions
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-$(rosversion -d)-turtlesim \
    ros-$(rosversion -d)-gazebo-ros \
    ros-$(rosversion -d)-gazebo-ros-pkgs \
    ros-$(rosversion -d)-gazebo-ros-control \
    ros-$(rosversion -d)-turtlebot3-gazebo \
    ros-$(rosversion -d)-turtlebot3-teleop \
    ros-$(rosversion -d)-turtlebot3-description \
    locales mesa-utils mesa-utils-extra \
    libgl1-mesa-dri libgl1-mesa-glx libglu1-mesa libglx-mesa0 \
    xvfb cargo rustc tmux git nano docker \
    python3-empy python3-defusedxml python3-pip python3.9 python3.9-venv \
 && rm -rf /var/lib/apt/lists/*

# Install Python 3.10 from source
RUN apt-get update && apt-get install -y \
        build-essential \
        libssl-dev zlib1g-dev libncurses5-dev libncursesw5-dev \
        libreadline-dev libsqlite3-dev libgdbm-dev libdb5.3-dev \
        libbz2-dev libexpat1-dev liblzma-dev tk-dev libffi-dev wget \
    && wget https://www.python.org/ftp/python/3.10.15/Python-3.10.15.tgz \
    && tar xvf Python-3.10.15.tgz \
    && cd Python-3.10.15 \
    && ./configure --enable-optimizations \
    && make -j$(nproc) \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.10.15 Python-3.10.15.tgz

# ------------------------------------------------------------
# #####CHECK##### Install ROS2 Foxy alongside ROS1 Noetic
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-rosbag2 \
    ros-foxy-rosbag2-storage-default-plugins \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# #####CHECK##### Install ROS2 Humble alongside ROS1 Noetic
# ------------------------------------------------------------
#RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
#    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
#    apt-get update && apt-get install -y \
#    ros-humble-desktop \
#    ros-humble-rosbag2 \
#    ros-humble-rosbag2-storage-default-plugins \
#    python3-colcon-common-extensions \
# && rm -rf /var/lib/apt/lists/*

# ============================================================
#  ROSA virtual environment (Python 3.9)
# ============================================================
RUN python3.9 -m venv /opt/venv/rosa && \
    /opt/venv/rosa/bin/pip install -U pip setuptools wheel

# Rust/Cargo path (used by flash-attn / torch)
ENV PATH="/root/.cargo/bin:${PATH}"

# ------------------------------------------------------------
# ROSA dependencies
# ------------------------------------------------------------
RUN /opt/venv/rosa/bin/pip install -U python-dotenv catkin_tools rospkg empy

RUN rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "alias start='catkin build && source devel/setup.bash && roslaunch waffle_agent agent.launch'" >> /root/.bashrc && \
    echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> /root/.bashrc

# Copy ROSA code and install it
COPY . /app/
WORKDIR /app/

RUN /bin/bash -c 'if [ "$DEVELOPMENT" = "true" ]; then \
        /opt/venv/rosa/bin/pip install --user -e .; \
    else \
        /opt/venv/rosa/bin/pip install -U jpl-rosa>=1.0.7; \
    fi'

# ============================================================
#  REMEMBR + VILA virtual environment (Python 3.10)
# ============================================================
RUN python3.10 -m venv /opt/venv/remembr && \
    /opt/venv/remembr/bin/python -m ensurepip && \
    /opt/venv/remembr/bin/pip install -U pip setuptools wheel

# Copy and install REMEMBR (assuming its source is in /app/remembr)
WORKDIR /app/remembr

# Install REMEMBR dependencies
RUN /opt/venv/remembr/bin/pip install -r requirements.txt

# Install VILA
RUN /opt/venv/remembr/bin/pip install -e . && \
    mkdir -p deps && cd deps && \
    git clone https://github.com/NVlabs/VILA.git && \
    cd VILA && \
    /opt/venv/remembr/bin/pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.8/flash_attn-2.5.8+cu122torch2.3cxx11abiFALSE-cp310-cp310-linux_x86_64.whl &&\
    /opt/venv/remembr/bin/pip install -e . && \
    /opt/venv/remembr/bin/pip install -e ".[train]" && \
    /opt/venv/remembr/bin/pip install -e ".[eval]" && \
    /opt/venv/remembr/bin/pip install -U transformers==4.46.0

# Patch DeepSpeed as from Remembr instructions (still valid for current VILA)
#RUN site_pkg_path=$(/opt/venv/remembr/bin/python -c 'import site; print(site.getsitepackages()[0])') && \
#    cp -rv /app/remembr/deps/VILA/llava/train/deepspeed_replace/* $site_pkg_path/deepspeed/ || true

# ============================================================
#  Convenience environment variables
# ============================================================
ENV ROSA_ENV=/opt/venv/rosa \
    REMEMBR_ENV=/opt/venv/remembr

# Helpers to switch envs in interactive shells
RUN echo '#!/bin/bash\nsource /opt/venv/rosa/bin/activate'   > /app/use_rosa && \
    echo '#!/bin/bash\nsource /opt/venv/remembr/bin/activate' > /app/use_remembr && \
    chmod +x /app/use_rosa /app/use_remembr

# ============================================================
#  Default runtime: start ROS core and open shell
# ============================================================
WORKDIR /app
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    roscore > /dev/null 2>&1 & \
    sleep 5 && \
    echo 'Run runwaffletester.sh to launch the Gazebo test environment, then run `start` to build and launch the rosa_waffle_bot controller.' && \
    /bin/bash"]




# TO CHECK: DEPENDENCIES ORDER:
#      Attempting uninstall: gradio
#    Found existing installation: gradio 3.35.2
#    Uninstalling gradio-3.35.2:
#      Successfully uninstalled gradio-3.35.2
#ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
#vila 2.0.0 requires accelerate==0.34.2, but you have accelerate 0.33.0 which is incompatible.
#vila 2.0.0 requires gradio==3.35.2, but you have gradio 3.50.2 which is incompatible.
#vila 2.0.0 requires gradio_client==0.2.9, but you have gradio-client 0.6.1 which is incompatible.
#vila 2.0.0 requires openai==1.8.0, but you have openai 1.109.1 which is incompatible.
#Successfully installed SQLAlchemy-2.0.44 accelerate-0.33.0 aiofiles-23.2.1 altair-5.5.0 async-timeout-4.0.3 dataclasses-json-0.6.7 gradio-3.50.2 gradio-client-0.6.1 greenlet-3.2.4 grpcio-1.76.0 importlib-resources-6.5.2 jiter-0.12.0 jsonpatch-1.33 jsonpointer-3.0.0 langchain-0.2.17 langchain-community-0.2.19 langchain-core-0.2.43 langchain-text-splitters-0.2.4 langchain_huggingface-0.0.3 langchain_nvidia_ai_endpoints-0.2.2 langchain_openai-0.1.25 langgraph-0.4.0 langgraph-checkpoint-2.1.2 langgraph-prebuilt-0.1.8 langgraph-sdk-0.2.9 langsmith-0.1.147 markupsafe-2.1.5 marshmallow-3.26.1 mypy-extensions-1.1.0 openai-1.109.1 ormsgpack-1.12.0 packaging-24.2 pillow-10.4.0 pydantic-1.10.18 pymilvus-2.6.3 python-dotenv-1.2.1 requests-toolbelt-1.0.0 sentence-transformers-