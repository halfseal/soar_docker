FROM osrf/ros:noetic-desktop-full-focal

# ros
RUN apt-get update \
    && apt-get install -y curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt install -y python3-rosdep \
    && apt install -y python3-rosinstall \
    && apt install -y python3-rosinstall-generator \
    && apt install -y python3-wstool \
    && apt install -y build-essential

# unitree
RUN apt-get install ros-noetic-pcl-conversions
RUN apt-get install libeigen3-dev

RUN apt-get install -y vim \
    && apt-get install -y git \
    && apt-get install -y python3-pip \
    && apt-get install -y nautilus \ 
    && apt-get install -y ros-noetic-catkin \
    && apt-get install -y python3-catkin-tools
RUN rm -rf /var/lib/apt/lists/*

# pixhawk, qgroundcontrol
RUN apt-get update \
    && apt-get install -y protobuf-compiler libeigen3-dev libopencv-dev \
    && apt-get remove -y modemmanager \
    && apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && apt-get install -y libqt5gui5 \
    && apt-get install -y libfuse2 \
    && apt-get install -y apt-utils \
    && apt-get install -y libpulse-dev \
    && apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
RUN rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]

# unilidar
WORKDIR /root
RUN git clone https://github.com/unitreerobotics/unilidar_sdk.git
WORKDIR /root/unilidar_sdk/unitree_lidar_ros
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make

# pointlio
WORKDIR /root
RUN mkdir -p /root/catkin_point_lio_unilidar/src 
WORKDIR /root/catkin_point_lio_unilidar/src
RUN git clone https://github.com/unitreerobotics/point_lio_unilidar.git
WORKDIR /root/catkin_point_lio_unilidar
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make

# px4
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
ENV PATH="/root/.local/bin:${PATH}"
RUN pip install --upgrade numpy
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
WORKDIR /root/PX4-Autopilot

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y ros-noetic-mavros
RUN apt-get install -y ros-noetic-mavros-extras 
RUN apt-get install -y ros-noetic-mavros-msgs
RUN apt-get install -y python3-lxml
RUN apt-get install -y python3-future
RUN rm -rf /var/lib/apt/lists/*

# mavros
WORKDIR /root
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN bash ./install_geographiclib_datasets.sh
RUN rm -rf /root/install_geographiclib_datasets.sh

# mavros example
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN catkin init
RUN wstool init /root/catkin_ws/src
RUN rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
RUN rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
RUN wstool merge -t src /tmp/mavros.rosinstall
RUN wstool update -t src -j4
RUN rosdep install --from-paths src --ignore-src -y
RUN ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
RUN source /opt/ros/noetic/setup.bash \
    && catkin build
RUN source /root/catkin_ws/devel/setup.bash

# building px4_sitl gazebo-classic (refer to this: https://docs.px4.io/main/en/dev_setup/building_px4.html)
WORKDIR /root/PX4-Autopilot
COPY makesitl.bash /root/PX4-Autopilot
RUN bash makesitl.bash

# offboard controll example (refer https://docs.px4.io/main/ko/ros/mavros_offboard_python.html)
# offboard_py
WORKDIR /root/catkin_ws/src
RUN catkin_create_pkg offboard_py rospy
WORKDIR /root/catkin_ws
RUN source /opt/ros/noetic/setup.bash \
    && catkin build
RUN source /root/catkin_ws/devel/setup.bash
WORKDIR /root/catkin_ws/src/offboard_py
RUN mkdir scripts
WORKDIR /root/catkin_ws/src/offboard_py/scripts
COPY offb_node.py /root/catkin_ws/src/offboard_py/scripts
RUN chmod +x offb_node.py

# start_offb.launch
WORKDIR /root/catkin_ws/src/offboard_py
RUN mkdir launch
WORKDIR /root/catkin_ws/src/offboard_py/launch
COPY start_offb.launch /root/catkin_ws/src/offboard_py/launch
RUN echo "source ~/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default" >> /root/.bashrc && \
    echo 'export ROS_PACKAGE_PATH=~/PX4-Autopilot:$ROS_PACKAGE_PATH' >> /root/.bashrc && \
    echo 'export ROS_PACKAGE_PATH=~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ROS_PACKAGE_PATH' >> /root/.bashrc && \
    echo 'export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH' >> /root/.bashrc

WORKDIR /root/catkin_ws

# RUN useradd -ms /bin/bash soar
# RUN chown -R soar:soar /root
# USER soar
# WORKDIR /root
# RUN curl -O https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
# RUN chmod +x ./QGroundControl.AppImage

# USER soar
# WORKDIR /root
