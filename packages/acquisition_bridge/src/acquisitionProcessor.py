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
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, LightSensor, LanePose
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils import get_duckiefleet_root
import yaml
import threading


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
        self.wrong_size = False
        self.currentLux = 0
        self.newLuxData = False

        if self.is_autobot:
            self.trim = 0.0
            self.gain = 1.0
            self.readParamFromFile()
            self.acq_topic_wheel_command = "wheels_driver_node/wheels_cmd"
            self.wheel_command_subscriber = rospy.Subscriber(
                '/'+self.veh_name+'/'+self.acq_topic_wheel_command, WheelsCmdStamped, self.wheel_command_callback,  queue_size=5)
            self.emergency_stop_publisher = rospy.Publisher(
                "/"+self.veh_name+"/wheels_driver_node/emergency_stop", BoolStamped, queue_size=1)
            self.lane_pose_publisher = rospy.Publisher(
                "/"+self.veh_name+"/LanePose", LanePose, queue_size=1)
            self.wheels_cmd_msg_list = []
            self.wheels_cmd_lock = threading.Lock()
        else:
            self.light_sensor_subscriber = rospy.Subscriber(
                '/'+self.veh_name+'/light_sensor_node/sensor_data', LightSensor, self.cb_light_sensor, queue_size=1)

        self.subscriberCompressedImage = rospy.Subscriber(
            '/'+self.veh_name+'/'+self.acq_topic_raw, CompressedImage, self.camera_image_process, queue_size=30, buff_size=4000000)

        self.subscriberCameraInfo = rospy.Subscriber(
            '/'+self.veh_name+'/'+"camera_node/camera_info", CameraInfo, self.camera_info,   queue_size=1)

    def getFilePath(self, name):
        return get_duckiefleet_root()+'/calibrations/kinematics/' + name + ".yaml"

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." % (
                self.node_name, fname))
            fname = self.getFilePath("default")
            if not os.path.isfile(fname):
                print("Nothing in %s" % fname)
                return

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.logger.warning("could not open %s " % in_file)
                # rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" % (
                #     self.node_name, fname, exc))
                # rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        gain = yaml_dict.get("gain")
        if gain is not None:
            self.gain = gain
        trim = yaml_dict.get("trim")
        if trim is not None:
            self.trim = trim

    def wheel_command_callback(self, wheels_cmd):
        current_time = time.time()
        wheels_cmd.header.stamp.secs = int(current_time)
        wheels_cmd.header.stamp.nsecs = int(
            (current_time - wheels_cmd.header.stamp.secs) * 10**9)
        # Decalibrate the wheel command for odometry
        k_r_inv = self.gain + self.trim
        k_l_inv = self.gain - self.trim

        wheels_cmd.vel_right = self.gain * wheels_cmd.vel_right / k_r_inv
        wheels_cmd.vel_left = self.gain * wheels_cmd.vel_left / k_l_inv

        with self.wheels_cmd_lock:
            self.wheels_cmd_msg_list.append(wheels_cmd)

    def add_image_to_list(self, image):
        with self.listLock:
            self.imageCompressedList.append(image)

    def camera_image_process(self, currRawImage):
        t = threading.Thread(target=self.add_image_to_list,
                             args=(currRawImage,))
        t.start()

        if not self.is_autobot:
            currentStamp = currRawImage.header.stamp.secs + \
                currRawImage.header.stamp.nsecs*10**(-9)
            if currentStamp - self.lastProcessedStamp > self.processPeriod:
                if self.cvImage is not None and not self.wrong_size:
                    self.previousCvImage = self.cvImage
                self.cvImage = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(
                    currRawImage, desired_encoding='mono8'), (0, 0), fx=0.10, fy=0.10)
                self.cvImage = self.cvImage[10:-10, 15:-15]
                if self.previousCvImage is not None:
                    if(self.previousCvImage.shape != self.cvImage.shape):
                        self.logger.info("Previous image and current image dont have the same dimensions : %s VS %s" % (
                            str(self.previousCvImage.shape), str(self.cvImage.shape)))
                        self.publishImages = True
                        self.wrong_size = True
                    else:
                        self.wrong_size = False

                        maskedImage = cv2.absdiff(
                            self.previousCvImage, self.cvImage)
                        _, maskedImage = cv2.threshold(
                            maskedImage, 30, 255, cv2.THRESH_BINARY)
                        self.maskNorm = cv2.norm(maskedImage)
                        if self.debug:
                            self.mask = self.bridge.cv2_to_compressed_imgmsg(
                                maskedImage, dst_format='png')
                            self.mask.header = currRawImage.header
                        self.logger.info("mask norm is %s" %
                                         str(self.maskNorm))

                        if self.maskNorm > 500:
                            self.publishImages = True
                        else:
                            self.publishImages = False
                            self.lastEmptyImageStamp = currentStamp
                            self.flushbuffer()
                        self.newMaskNorm = True
                self.lastProcessedStamp = currentStamp
        else:
            self.publishImages = True

    def camera_info(self, camera_info):
        self.lastCameraInfo = camera_info

    def cb_light_sensor(self, data):
        self.currentLux = data.real_lux
        self.newLuxData = True

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
                        try:
                            outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                                block=True,
                                                timeout=0.5)
                        except Exception:
                            self.logger.info(
                                "Timeout on outputqueue reached when trying to add images")
                            break
                    self.imageCompressedList = []

            outputDict = dict()
            if self.is_autobot:
                with self.wheels_cmd_lock:
                    for wheels_cmd in self.wheels_cmd_msg_list:
                        outputDict = {"wheels_cmd": wheels_cmd}
                        try:
                            outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                                block=True,
                                                timeout=0.5)
                        except Exception:
                            self.logger.info(
                                "Timeout on outputqueue reached when trying to add wheel commands")
                            break
                    self.wheels_cmd_msg_list = []
            else:
                outputDict = dict()
                if self.newMaskNorm:
                    self.newMaskNorm = False
                    outputDict['maskNorm'] = self.maskNorm

                if self.newLuxData:
                    self.newLuxData = False
                    outputDict['currentLux'] = self.currentLux

                if outputDict:
                    try:
                        outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                            block=True,
                                            timeout=0.5)
                    except Exception:
                        self.logger.info(
                            "Timeout on outputqueue reached when trying to add currentlux or masknorm")

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
                if "LanePose" in incomingData:
                    LanePoseMsg = incomingData["LanePose"]
                    print("Duckie received lane pose")
                    self.lane_pose_publisher.publish(LanePoseMsg)


            except KeyboardInterrupt:
                raise(Exception("Exiting"))
            except Queue.Empty:
                pass
            except Exception as e:
                self.logger.warning("Exception: %s" % str(e))
                pass
