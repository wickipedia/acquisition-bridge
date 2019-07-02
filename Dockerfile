FROM duckietown/rpi-ros-kinetic-base:master19

RUN [ "cross-build-start" ]



# Install

RUN apt-get update && apt-get install -y --allow-unauthenticated --no-install-recommends ros-kinetic-rospy python-pip && apt-get clean
RUN pip install pathos multiprocessing-logging

# Argumens for where the different places from which we will get files from are
ARG acquisition_src_dir=ros-acquisition-bridge/src

# Create a directory to store the Python files and the April tag library
RUN mkdir /acquisition_node

# Copy the Python files
COPY ${acquisition_src_dir}/acquisition_node/acquisitionProcessor.py /acquisition_node
COPY ${acquisition_src_dir}/acquisition_node/serverSidePublisher.py /acquisition_node
COPY ${acquisition_src_dir}/acquisition_node/acquire_and_publish.py /acquisition_node
COPY ${acquisition_src_dir}/acquisition_node/set_environment.sh /acquisition_node
RUN chmod +x /acquisition_node/*.py
RUN chmod +x /acquisition_node/*.sh

# Setup the duckietown_msgs
RUN mkdir -p /catkin-ws/src
COPY ${acquisition_src_dir}/duckietown_msgs /catkin-ws/src/duckietown_msgs
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /catkin-ws/; catkin_make"

# FLAG : put to true for autobots (continuous image publishing no matter what)
ENV SKIP_BACKGROUND_SUBSTRACTION False
ENV IS_AUTOBOT False

RUN [ "cross-build-end" ]


# Start the processes
CMD /bin/bash -c "cd /acquisition_node; source /opt/ros/kinetic/setup.bash; source /catkin-ws/devel/setup.bash; source ./set_environment.sh; python acquire_and_publish.py"
