FROM ros:melodic
MAINTAINER Daiki Hayashi <hayashi.daiki@hdwlab.co.jp>
ENV LANG="en_US.UTF-8"
SHELL ["/bin/bash", "-c"]

# Add setup.bash to path.sh
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /opt/path.sh

# Apt update and rosdep update
RUN apt update && rosdep update

# Install rosbridge
RUN bash -c '\
  source /root/.bashrc \
  && source /opt/path.sh \
  && mkdir -p /opt/catkin_workspace \
  && cd /opt/catkin_workspace \
  && git clone --depth=1 -b 0.11.5 https://github.com/RobotWebTools/rosbridge_suite.git src \
  && cd src \
  && catkin_init_workspace \
  && cd .. \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make \
  && echo "source /opt/catkin_workspace/devel/setup.bash" >> /opt/path.sh \
'

# Install rosbridge-rosbag-player
RUN mkdir -p /opt/rosbridge-rosbag-player/src
COPY ./src /opt/rosbridge-rosbag-player/src
RUN bash -c '\
  source /root/.bashrc \
  && source /opt/path.sh \
  && cd /opt/rosbridge-rosbag-player/src \
  && catkin_init_workspace \
  && cd .. \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make \
  && echo "source /opt/rosbridge-rosbag-player/devel/setup.bash" >> /opt/path.sh \
'

# Remove unnecessary files
RUN apt autoremove -y && apt clean -y

# Set entry-point
COPY ./entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Copy ros-launch file
COPY ./player.launch /player.launch

# Set default command
CMD ["roslaunch", "/player.launch"]
