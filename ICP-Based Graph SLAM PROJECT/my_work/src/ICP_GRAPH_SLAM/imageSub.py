#!/usr/bin/env python
import sys
from PIL import Image as pilImage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt


import numpy as np
import imagehash
import keyboard
import string
import rospy
import cv2



def process_image(msg):
	
	
	bridge = CvBridge()
	orig = bridge.imgmsg_to_cv2(msg, "bgr8")
	orig2 = cv2.cvtColor(orig, cv2.COLOR_BGR2RGB)
	origPil = pilImage.fromarray(orig2)
	imageHash = imagehash.average_hash(origPil)
	hashInt = int("0x" + str(imageHash).upper(),16)
	rospy.loginfo('image hash: %d',hashInt)
	cv2.imshow('image', orig)
	
def start_node():
	rospy.init_node('detect_feature')
	rospy.loginfo('detect_feature node started')
	rospy.Subscriber("/camera/image_raw", Image, process_image)
	rospy.spin()

if __name__ == '__main__':
    start_node()






