#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import cPickle as pickle
import os
import Queue
import time
import threading
import cv2
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from cv_bridge import CvBridge, CvBridgeError


class acquisitionProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """

    def __init__(self, logger, is_autobot):
        self.logger = logger
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        # Get the environment variables
        self.acq_topic_raw = 'camera_node/image/compressed'

        # Flag to skip the Raw processing step and always publish (i.e. for Duckiebots)
        self.is_autobot = is_autobot
        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('acquisition_processor',
                        anonymous=True, disable_signals=True)
        self.subscriberCompressedImage = rospy.Subscriber(
            '/'+self.veh_name+'/'+self.acq_topic_raw, CompressedImage, self.camera_image_process,  queue_size=30)

        self.subscriberCameraInfo = rospy.Subscriber(
            '/'+self.veh_name+'/'+"camera_node/camera_info", CameraInfo, self.camera_info,  queue_size=1)

        if self.is_autobot:
            self.acq_topic_wheel_command = "wheels_driver_node/wheels_cmd"
            self.wheel_command_subscriber = rospy.Subscriber(
                '/'+self.veh_name+'/'+self.acq_topic_wheel_command, WheelsCmdStamped, self.wheel_command_callback,  queue_size=5)
            self.emergency_stop_publisher = rospy.Publisher(
                "/"+self.veh_name+"/wheels_driver_node/emergency_stop", BoolStamped, queue_size=1)
            self.wheels_cmd_msg_list = []
            self.wheels_cmd_lock = threading.Lock()

        self.timeLastPub_poses = 0
        self.bridge = CvBridge()
        self.mask = None
        self.cvImage = None
        self.newMaskNorm = False
        self.maskNorm = 0
        self.previousCvImage = None
        self.logger.info('Acquisition processor is set up.')
        self.debug = False
        self.imageCompressedList = []
        self.publishImages = True
        self.lastEmptyImageStamp = 0
        self.listLock = threading.Lock()
        self.lastCameraInfo = None
        self.lastProcessedStamp = 0.0
        self.processPeriod = 0.5

    def wheel_command_callback(self, wheels_cmd):
        current_time = time.time()
        wheels_cmd.header.stamp.secs = int(current_time)
        wheels_cmd.header.stamp.nsecs = int(
            (current_time - wheels_cmd.header.stamp.secs) * 10**9)

        with self.wheels_cmd_lock:
            self.wheels_cmd_msg_list.append(wheels_cmd)

    def camera_image_process(self, currRawImage):
        with self.listLock:
            self.imageCompressedList.append(currRawImage)

        if not self.is_autobot:
            currentStamp = currRawImage.header.stamp.secs + \
                currRawImage.header.stamp.nsecs*10**(-9)
            if currentStamp - self.lastProcessedStamp > self.processPeriod:
                if self.cvImage is not None:
                    self.previousCvImage = self.cvImage
                self.cvImage = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(
                    currRawImage, desired_encoding='mono8'), (0, 0), fx=0.10, fy=0.10)
                self.cvImage = self.cvImage[10:-10, 15:-15]
                if self.previousCvImage is not None:
                    maskedImage = cv2.absdiff(
                        self.previousCvImage, self.cvImage)
                    _, maskedImage = cv2.threshold(
                        maskedImage, 30, 255, cv2.THRESH_BINARY)
                    self.maskNorm = cv2.norm(maskedImage)
                    self.newMaskNorm = True
                    if self.debug:
                        self.mask = self.bridge.cv2_to_compressed_imgmsg(
                            maskedImage, dst_format='png')
                        self.mask.header = currRawImage.header
                    self.logger.info("mask norm is %s" % str(self.maskNorm))

                    if self.maskNorm > 500:
                        self.publishImages = True
                    else:
                        self.publishImages = False
                        self.lastEmptyImageStamp = currentStamp
                        self.flushbuffer()
                self.lastProcessedStamp = currentStamp
        else:
            self.publishImages = True

    def camera_info(self, camera_info):
        self.lastCameraInfo = camera_info

    def flushbuffer(self):
        with self.listLock:
            while self.imageCompressedList:
                image = self.imageCompressedList[0]
                currentImageStamp = image.header.stamp.secs + \
                    image.header.stamp.nsecs*10**(-9)
                if currentImageStamp < self.lastEmptyImageStamp:
                    self.imageCompressedList.remove(image)
                else:
                    break

    def liveUpdate(self, outputDictQueue, inputDictQueue, quitEvent):
        """
        Runs constantly and processes new data as it comes.
        """
        while not quitEvent.is_set():
            # Check if the last image data was not yet processed and if it's time to process it (in order to sustain the deisred update rate)
            outputDict = dict()
            if self.publishImages:
                with self.listLock:
                    for image in self.imageCompressedList:
                        # Collect latest ros_data
                        outputDict['imageStream'] = image
                        if self.lastCameraInfo is not None:
                            outputDict['cameraInfo'] = self.lastCameraInfo
                        if self.mask is not None and self.debug:
                            outputDict['mask'] = self.mask
                    self.imageCompressedList = []

            if self.is_autobot:
                with self.wheels_cmd_lock:
                    for wheels_cmd in self.wheels_cmd_msg_list:
                        outputDict = {"wheels_cmd": wheels_cmd}
                        outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                            block=True,
                                            timeout=None)
                    self.wheels_cmd_msg_list = []
            if self.newMaskNorm:
                self.newMaskNorm = False
                outputDict['maskNorm'] = self.maskNorm
            if outputDict:
                outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                    block=True,
                                    timeout=None)
            else:
                time.sleep(0.05)
            try:
                newQueueData = inputDictQueue.get(block=False)
                incomingData = pickle.loads(newQueueData)
                if "requestImage" in incomingData:
                    self.logger.info("Request received")
                    imgMsg = incomingData["requestImage"]
                    self.publishImages = True
                if "toggleEmergencyStop" in incomingData:
                    stopMsg = BoolStamped()
                    stopMsg.data = incomingData["toggleEmergencyStop"]
                    self.emergency_stop_publisher.publish(stopMsg)
                    self.logger.info("Emergency stop toggled")

            except KeyboardInterrupt:
                raise(Exception("Exiting"))
            except Queue.Empty:
                pass
            except Exception as e:
                self.logger.warning("Exception: %s" % str(e))
                pass
