FROM ubuntu:jammy

ARG ARG1=0
ARG ROS_DISTRO=humble

#Install ROS2 on ubuntu jammy
RUN locale
RUN apt-get update && apt-get install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
 
RUN locale  # verify settings
 
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe
 
RUN apt-get update && apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install \
ros-${ROS_DISTRO}-desktop ros-dev-tools python3-colcon-common-extensions -y 

# BT.ROS2 related deps
RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-humble-generate-parameter-library -y

# Additionally ROS-pkgs
# RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-${ROS_DISTRO}-gazebo* -y
# RUN apt-get install ros-${ROS_DISTRO}-rclcpp-cascade-lifecycle \
# ros-${ROS_DISTRO}-test-msgs ros-${ROS_DISTRO}-nav2-msgs -y

# Boost & yaml-cpp
RUN apt-get install libboost-dev libyaml-dev -y

# Terminator
RUN apt-get install terminator -y


# BT.CPP dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install libzmq3-dev libsqlite3-dev -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install libncurses5-dev libncursesw5-dev -y

RUN mkdir -p /eut_bt_forest_ws/src
WORKDIR /eut_bt_forest_ws/src

# BT.CPP v 4.6.2
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git --branch 4.6.2 --single-branch
RUN git clone https://github.com/DavidCalero1/rosx_introspection.git

# REMAP THE v4_pub from EUT gitlab
# Successor of ros type introspection, ros2 type introspection and ros msg parser (to be tested) 
# RUN git clone https://github.com/facontidavide/rosx_introspection.git

# RUN wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-linux-installer.run 
#RUN chmod +x Groot2-v1.5.2-linux-installer.run

#Groot dependencies
RUN apt-get install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev -y

WORKDIR /eut_bt_forest_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash;sudo rosdep init;rosdep update; rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO};"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash;colcon build --symlink-install"

SHELL ["/bin/bash", "-c"]
RUN echo source /opt/ros/humble/setup.bash >> /root/.bashrc
RUN echo source /eut_bt_forest_ws/install/setup.bash >> /root/.bashrc
RUN echo source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash >> /root/.bashrc

CMD ["cd /eut_bt_forest_ws"]
 