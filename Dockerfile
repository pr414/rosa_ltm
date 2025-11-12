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
# COPY remembr/ .    # uncomment if copying from build context
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
