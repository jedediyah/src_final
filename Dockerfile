FROM ros:indigo-ros-base

# add ihmc messages
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    python-catkin-tools \
    ros-indigo-catkin \
    ros-indigo-ihmc-msgs \
    ros-indigo-rosbag \
    ros-indigo-cv-bridge \
    ros-indigo-tf \
    ros-indigo-tf2 \
 && rm -rf /var/lib/apt/lists/*

# clone srcsim
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim

# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
 && catkin build

# include bag file with footsteps preprogrammed
#ADD footsteps_2017-05-02-14-40-59.bag ${WS}/
#RUN echo "while rosbag play ${WS}/footsteps_2017-05-02-14-40-59.bag && python ${WS}/src/srcsim/scripts/rossleep.py 8; do date; done" \
#  > do_footsteps.bash

EXPOSE 8000
EXPOSE 8001
EXPOSE 5005/udp
EXPOSE 6006/udp
EXPOSE 4004
ENV ROS_MASTER_URI http://127.0.0.1:8001

# startup script
ADD communications.py communications.py
ADD cv2_addons.py cv2_addons.py
ADD echoserver.py echoserver.py
ADD keyboard_teleop.py keyboard_teleop.py
ADD listalgebra.py listalgebra.py
ADD perception.py perception.py
ADD point_cloud2.py point_cloud2.py
ADD valcontrol.py valcontrol.py
ADD val_walk_controller.py val_walk_controller.py
ADD whalers_field_comms.py whalers_field_comms.py
ADD whalers_field_control.py whalers_field_control.py
ADD wfield_receiver.py wfield_receiver.py 
# simple HTTP server and a roscore
ADD startup.bash startup.bash
CMD ["./startup.bash"]
