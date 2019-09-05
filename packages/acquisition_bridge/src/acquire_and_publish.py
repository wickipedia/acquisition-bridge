#!/usr/bin/env python

import socket
from serverSidePublisher import publishingProcessor
from acquisitionProcessor import acquisitionProcessor
import math
import numpy as np
import rospy
import rosbag
from sensor_msgs.msg import CompressedImage

import sys
import os
import time

import multiprocessing
import logging
import traceback
logger = multiprocessing.log_to_stderr()
logger.setLevel(logging.INFO)


def get_ip():
    # from https://stackoverflow.com/questions/166506/finding-local-ip-addresses-using-pythons-stdlib

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


# IMPORT THE ENVIRONMENT VARIABLES (DEPENDING ON THE MODE)
ACQ_ROS_MASTER_URI_DEVICE_IP = get_ip()
ACQ_ROS_MASTER_URI_DEVICE_PORT = "11311"

LAB_ROS_MASTER_IP = os.getenv('LAB_ROS_MASTER_IP', "")
LAB_ROS_MASTER_PORT = "11311"


# Define the two concurrent processes:
def runDeviceSideProcess(ROS_MASTER_URI, quitEvent, is_autobot):
    """
    Receive and process data from the remote device (Duckiebot or watchtower).
    """
    logger.info('Device side processor starting')

    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    ap = acquisitionProcessor(logger, is_autobot)
    ap.liveUpdate(outputDictQueue, inputDictQueue, quitEvent)


def runServerSideProcess(ROS_MASTER_URI, quitEvent, is_autobot):
    """
    Publush the processed data to the ROS Master that the graph optimizer uses.
    """
    logger.info('Server side processor starting')
    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    pp = publishingProcessor(logger, is_autobot)
    pp.publishOnServer(outputDictQueue, inputDictQueue,
                       quitEvent)


if __name__ == '__main__':
    """
    Starts the two processes and sets up their termination.
    """

    print("Image and odometry acquision, processing and publishing setting up")

    # Event to terminate the two processes
    quitEvent = multiprocessing.Event()

    ros_master_uri_server = "http://"+LAB_ROS_MASTER_IP + \
        ":"+LAB_ROS_MASTER_PORT
    ros_master_uri_device = "http://"+ACQ_ROS_MASTER_URI_DEVICE_IP + \
        ":"+ACQ_ROS_MASTER_URI_DEVICE_PORT

    node_name = rospy.get_name()
    veh_name = node_name.split("/")[1]

    is_autobot = bool(rospy.get_param(
        "/"+veh_name+"/acquisition_bridge/is_autobot", default=True))

    # outputDictQueue is used to pass data between the two processes
    outputDictQueue = multiprocessing.Queue(maxsize=100)
    inputDictQueue = multiprocessing.Queue(maxsize=10)

    # Start the processes

    deviceSideProcess = multiprocessing.Process(target=runDeviceSideProcess,
                                                args=(
                                                    ros_master_uri_device, quitEvent, is_autobot),
                                                name="deviceSideProcess")

    serverSideProcess = multiprocessing.Process(target=runServerSideProcess,
                                                args=(
                                                    ros_master_uri_server, quitEvent, is_autobot),
                                                name="serverSideProcess")
    deviceSideProcess.start()
    serverSideProcess.start()

    # Exit if any of the two processes exits:
    while True:
        # print("Current approx. size %d " % outputDictQueue.qsize())

        if not deviceSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            # Wait until the queue is processed
            while not outputDictQueue.empty() and serverSideProcess.is_alive():
                time.sleep(1)
            outputDictQueue.close()
            # Give time for submitting the last message to the server
            serverSideProcess.join()
            time.sleep(0.5)
            serverSideProcess.terminate()
            print("The device side process exited. Stopping everything.")
            sys.exit()

        if not serverSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            serverSideProcess.terminate()
            outputDictQueue.close()
            print("The server side process exited. Stopping everything.")
            sys.exit()

        time.sleep(1)
