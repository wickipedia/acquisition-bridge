FROM duckietown/dt-ros-commons:master19-arm32v7
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

# FLAG : put to true for autobots (continuous image publishing no matter what)
# ENV SKIP_BACKGROUND_SUBSTRACTION True
# ENV IS_AUTOBOT True

RUN [ "cross-build-end" ]



# Start the processes
CMD /bin/bash -c "cd /acquisition_node; source /opt/ros/kinetic/setup.bash; source /code/catkin_ws/devel/setup.bash; source ./set_environment.sh; python acquire_and_publish.py"
