#!/usr/bin/env python

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

import numpy as np
import math

from acquisitionProcessor import acquisitionProcessor
from serverSidePublisher import publishingProcessor

# IMPORT THE ENVIRONMENT VARIABLES (DEPENDING ON THE MODE)
ACQ_DEVICE_MODE = os.getenv('ACQ_DEVICE_MODE', 'live')
if ACQ_DEVICE_MODE != 'live':
    raise Exception("The environment variable ACQ_DEVICE_MODE should be 'live'. Received %s instead." % ACQ_MODE)
ACQ_SERVER_MODE = os.getenv('ACQ_SERVER_MODE', 'live')
if ACQ_SERVER_MODE != 'live':
    raise Exception("The environment variable ACQ_SERVER_MODE should be 'live'. Received %s instead." % ACQ_MODE)

ACQ_ROS_MASTER_URI_DEVICE_IP = os.getenv('ACQ_ROS_MASTER_URI_DEVICE_IP', "")
ACQ_ROS_MASTER_URI_DEVICE_PORT = os.getenv('ACQ_ROS_MASTER_URI_DEVICE_PORT', "")

ACQ_ROS_MASTER_URI_SERVER_IP = os.getenv('ACQ_ROS_MASTER_URI_SERVER_IP', "")
ACQ_ROS_MASTER_URI_SERVER_PORT = os.getenv('ACQ_ROS_MASTER_URI_SERVER_PORT', "")


# Define the two concurrent processes:
def runDeviceSideProcess(ROS_MASTER_URI, quitEvent):
    """
    Receive and process data from the remote device (Duckiebot or watchtower).
    """
    logger.info('Device side processor starting in LIVE mode')

    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    ap = acquisitionProcessor(logger, mode="live")
    ap.liveUpdate(outputDictQueue, inputDictQueue, quitEvent)

def runServerSideProcess(ROS_MASTER_URI, quitEvent):
    """
    Publush the processed data to the ROS Master that the graph optimizer uses.
    """
    logger.info('Server side processor starting in LIVE mode')
    os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
    pp = publishingProcessor(logger, mode="live")
    pp.publishOnServer(outputDictQueue, inputDictQueue, quitEvent, logger, mode='live')

if __name__ == '__main__':
    """
    Starts the two processes and sets up their termination.
    """

    print("Image and odometry acquision, processing and publishing setting up")

    # Event to terminate the two processes
    quitEvent = multiprocessing.Event()

    ros_master_uri_server = "http://"+ACQ_ROS_MASTER_URI_SERVER_IP+":"+ACQ_ROS_MASTER_URI_SERVER_PORT
    ros_master_uri_device = "http://"+ACQ_ROS_MASTER_URI_DEVICE_IP+":"+ACQ_ROS_MASTER_URI_DEVICE_PORT


    # outputDictQueue is used to pass data between the two processes
    outputDictQueue = multiprocessing.Queue(maxsize=40)
    inputDictQueue = multiprocessing.Queue(maxsize=10)

    # Start the processes

    deviceSideProcess = multiprocessing.Process(target=runDeviceSideProcess,
                                                args=(ros_master_uri_device,quitEvent),
                                                name="deviceSideProcess")
    deviceSideProcess.start()


    serverSideProcess = multiprocessing.Process(target=runServerSideProcess,
                                                args=(ros_master_uri_server,quitEvent),
                                                name="serverSideProcess")
    serverSideProcess.start()

    # Exit if any of the two processes exits:
    while True:
        # print("Current approx. size %d " % outputDictQueue.qsize())

        if not deviceSideProcess.is_alive():
            quitEvent.set()
            deviceSideProcess.terminate()
            #Wait until the queue is processed
            while not outputDictQueue.empty():
                time.sleep(0.1)
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

        time.sleep(0.2)
