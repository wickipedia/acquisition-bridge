#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
import cPickle as pickle
import os
import Queue
import time
import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError

class acquisitionProcessor():
    """
    Processes the data coming from a remote device (Duckiebot or watchtower).
    """
    def __init__(self, logger, mode='live'):

        self.mode = mode
        if self.mode != 'live' and self.mode != 'postprocessing':
            raise Exception("The argument mode should be 'live' or 'postprocessing'. Received %s instead." % self.mode)

        # Get the environment variables
        self.ACQ_DEVICE_NAME = os.getenv('ACQ_DEVICE_NAME', 'watchtower33')
        self.ACQ_TOPIC_RAW = os.getenv('ACQ_TOPIC_RAW', 'camera_node/image/compressed')
        self.ACQ_POSES_UPDATE_RATE = float(os.getenv('ACQ_POSES_UPDATE_RATE', 10)) #Hz

        # Initialize ROS nodes and subscribe to topics
        rospy.init_node('acquisition_processor', anonymous=True, disable_signals=True)
        self.subscriberCompressedImage = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+self.ACQ_TOPIC_RAW, CompressedImage, self.camera_image_process,  queue_size = 5)
        self.subscriberRawImage = rospy.Subscriber('/'+self.ACQ_DEVICE_NAME+'/'+"camera_node/image/raw", Image, self.camera_image_raw_process,  queue_size = 1)

        self.logger = logger
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
        self.publishImages = False
        self.lastEmptyImageStamp = 0
        self.listLock = threading.Lock()

    def camera_image_raw_process(self, currRawImage):
        if self.cvImage is not None:
            self.previousCvImage = self.cvImage
        self.cvImage = cv2.resize(self.bridge.imgmsg_to_cv2(currRawImage, desired_encoding='mono8'), (0,0), fx = 0.10, fy = 0.10)
        self.cvImage = self.cvImage[10:-10, 15:-15]
        if self.previousCvImage is not None:
            maskedImage = cv2.absdiff(self.previousCvImage, self.cvImage)
            _, maskedImage = cv2.threshold(maskedImage, 30, 255, cv2.THRESH_BINARY)
            self.maskNorm = cv2.norm(maskedImage)
            self.newMaskNorm = True
            if self.debug:
                self.mask = self.bridge.cv2_to_compressed_imgmsg(maskedImage, dst_format='png')
                self.mask.header=currRawImage.header
            if self.maskNorm > 500:
                self.publishImages = True
            else:
                self.publishImages = False
                self.lastEmptyImageStamp = currRawImage.header.stamp.secs+currRawImage.header.stamp.nsecs*10**(-9)
                self.flushbuffer()


    def camera_image_process(self, currImage):
        with self.listLock:
            self.imageCompressedList.append(currImage)

    def flushbuffer(self):
        with self.listLock:
            while self.imageCompressedList:
                image = self.imageCompressedList[0]
                currentImageStamp = image.header.stamp.secs+image.header.stamp.nsecs*10**(-9)
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
                        outputDict['imageStream']=image
                        if outputDict is not None:
                            outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                                block=True,
                                                timeout=None)
                            self.lastImageProcessed = True
                    self.imageCompressedList=[]
            if self.newMaskNorm:
                self.newMaskNorm = False
                outputDict['maskNorm']=self.maskNorm
                if self.mask is not None and self.debug:
                    outputDict['mask']=self.mask
                if outputDict is not None:
                    outputDictQueue.put(obj=pickle.dumps(outputDict, protocol=-1),
                                        block=True,
                                        timeout=None)

            try:
                newQueueData = inputDictQueue.get(block=False)
                incomingData = pickle.loads(newQueueData)
                if "requestImage" in incomingData:
                    self.logger.info("Request received")
                    imgMsg = incomingData["requestImage"]
                    self.publishImages = True

            except KeyboardInterrupt:
                raise( Exception("Exiting") )
            except Queue.Empty:
                pass
            except Exception as e:
                logger.warning("Exception: %s" % str(e))
                pass
