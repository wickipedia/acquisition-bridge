

To launch the acquisition bridge on a watchtower : 

laptop $ docker -H <HOSTNAME>.local run --name acquisition-bridge --network=host -dit -e LAB_ROS_MASTER_IP=<LAB_SERVER_IP> -e ROBOT_TYPE=watchtower duckietown/acquisition-bridge:devel20-arm32v7

To launch the acquisition bridge on a duckiebot : 

laptop $ docker -H <HOSTNAME>.local run --name acquisition-bridge --network=host -dit -e LAB_ROS_MASTER_IP=<LAB_SERVER_IP> duckietown/acquisition-bridge:devel20-arm32v7
