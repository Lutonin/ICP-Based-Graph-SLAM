#!/usr/bin/env python
import sys
import math
import rospy
import roslib
import numpy as np

from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan


from DataLoader_Edited import *



index = 0;


#initialize odometry

x,y,phi = (0,0,0)
xOld,yOld,phiOld = (0,0,0)
odomOut = open("odomOut.txt", "w")

#initialize laser scans

laserOut = open("laserOut.txt", "w")
currentPoints = np.array([])
oldPoints = np.array([])
laserIndex = 1

#initialize map
pointsOut = open("pointsOut.txt", "w")




	
def callback(laserScan):

	print("laser scan")
	global currentPoints, oldPoints, laserIndex
	laserScn = np.array(laserScan.ranges)
	laserScn[laserScn == np.inf] =0
	for scn in laserScn:
		laserOut.write(str(round(scn,3)) + " ")
	laserOut.write("\n")
	print(laserScan.angle_increment)
	pointsXY = RangeToPCL(laserScan)
	for point in pointsXY:
		pointsOut.write(str(round(point[0],2)) + "," + str(round(point[1],2)) + " ")
	pointsOut.write("\n")
	laserIndex += 1
	
def OdoCallback(odomMsg):

	#print("odometry scna")
	
	global odomOut
	global x,y,phi,xOld,yOld,phiOld
	position = odomMsg.pose.pose.position
	angle = odomMsg.pose.pose.orientation
	
	x = round(position.x,3)
	y = round(position.y,3)
	phi = round(angle.z,3)
	
	
	#print(str(x) + " " + str(y) + " " + str(phi) +" \n")
	odomOut.write(str(x) + " " + str(y) + " " + str(phi) +" \n")
	
def listener():
	rospy.init_node('laserListener', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.Subscriber("/odom", Odometry, OdoCallback)
	rospy.loginfo("node started")
	rospy.spin()
    

if __name__ == '__main__':
    	listener()
