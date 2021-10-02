#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys
import icp2
import scipy
import argparse
import time
import math
import g2o
import pose_graph

from matplotlib.patches import Ellipse
from sklearn.neighbors import KDTree
from sklearn.cluster import AffinityPropagation

def removeOutliers(pointCloud, threshold):
	newPointCloud = []
	index = 0
	tree  = KDTree(pointCloud, leaf_size = 2)
	dist, ind = tree.query(pointCloud[:], k = 5)
	for d in dist:
		if np.amax(d) < threshold:
			newPointCloud.append(pointCloud[index])
		index += 1
	return np.array(newPointCloud)

def preprocessingLocalMap(localMap,thresh):
	clustering2 = AffinityPropagation(damping = 0.8).fit(localMap)
	localMap_new = []
	for label in np.unique(clustering2.labels_):
		section = []
		for i in range(localMap.shape[0]):
			if(label == clustering2.labels_[i]):
				section.append(list(localMap[i]))
		section = averagePoints(np.array(section),thresh,0)
		section = averagePoints(np.array(section),thresh,1)
		localMap_new.extend(list(section))
	return np.array(localMap_new)

def averagePoints(localMap,thresh,axis):
    localMap_new = []
    ind = 0
    localMap = localMap[localMap[:,axis].argsort()]
    i = 0
    while(i<localMap.shape[0]):
        mainPoint = [localMap[i,0],localMap[i,1],1.0]
        checkIfFalse = False
        for j in range(i+1,localMap.shape[0]):         
            distance = np.sqrt(np.power(mainPoint[0] - localMap[j,0],2)+np.power(mainPoint[1] - localMap[j,1],2))
            if distance <= thresh:
            #   P = [(mainPoint[0]+localMap[j,0])/2, (mainPoint[1]+localMap[j,1])/2,1.0]
            #   mainPoint = P
		pass
            else:
                localMap_new.append(mainPoint)
                i = j 
                checkIfFalse = True
                break
        if(checkIfFalse == False):
            i +=1
    if not mainPoint in localMap_new:
	localMap_new.append(mainPoint)
    return np.array(localMap_new)
	
def getOdomTran(A,B,dx,pose):
	x, y, yaw = dx[0], dx[1], dx[2]
	init_pose = np.array([[np.cos(yaw), -np.sin(yaw), x],
                              [np.sin(yaw), np.cos(yaw), y],
                              [0, 0, 1]])

	tran, distances, iter, cov, _ = icp2.icp(
                    B, A, init_pose,
                    max_iterations=80, tolerance=0.0001)
		
	init_pose = tran
	pose = np.matmul(pose, tran)
		
	information = np.linalg.inv(cov)

	return pose,tran, distances,information
def getOdomTranPlane(A,B,dx,pose):
	x, y, yaw = dx[0], dx[1], dx[2]
	init_pose = np.array([[np.cos(yaw), -np.sin(yaw), x],
                              [np.sin(yaw), np.cos(yaw), y],
                              [0, 0, 1]])
def updateGraph(vertex_idx,pose,tran,cov,optimizer):
	optimizer.add_vertex(vertex_idx, g2o.SE2(g2o.Isometry2d(pose)))
	rk = g2o.RobustKernelDCS()
      	information = np.linalg.inv(cov)
       	optimizer.add_edge([vertex_idx-1, vertex_idx],
                        g2o.SE2(g2o.Isometry2d(tran)),
                           information, robust_kernel=rk)
	
	return optimizer,information
def XYtoHomo(B):
	Btemp = np.concatenate((B,np.ones((B.shape[0],1))),axis = 1)
	return Btemp
def PCTran(B,pose):
	B = XYtoHomo(B)
	Btemp = np.dot(B,pose.T)
	Btemp = Btemp[:,0:2]
	return Btemp
def pointScatter(B,color):
	plt.scatter(B[:,0],B[:,1],marker='+',s=1.0,c=color)
	return 1
def clearBackPoints(localMap,line,prevPose):
	p1 = [line[0,0],line[0,1]]
	p2 = [line[1,0], line[1,1]]
	slope = (p1[1]-p2[1])/(p1[0]-p2[0])
	if math.isinf(slope):
		slope = 1000
	b = p2[1] - slope*p2[0]
	
	up = 0
	if prevPose[1,0] > 0:
		up = 1
	else:
		up = -1
	tempLocalMap = np.array([[0,0]])
	for i,p in enumerate(localMap):
		if p[1] > slope*p[0]+b and up == 1:
			tempLocalMap = np.concatenate((tempLocalMap,[p[0:2]]),axis = 0)
		if p[1] < slope*p[0]+b and up == -1:
			tempLocalMap = np.concatenate((tempLocalMap,[p[0:2]]),axis = 0)	
	localMap = tempLocalMap
	return localMap

def readFile():
	numScan = 0
	lasersScan = []
	lasers = []
	odoms = []
	imageHash = []
	
	dataOut = open("dataOut.txt", "r")
	data = dataOut.read().split("\n")
	num_readings = 180
	for dataLine in data:
		
		if numScan % 3 == 0:
			laser = dataLine.split(" ")
			lsr = []
			for l in laser:
				if not l == "":
					lsr.append(float(l))
			index = np.arange(-90, 90+180/num_readings, 180/num_readings)
           		index = np.delete(index, num_readings//2)
           		converted_scans = []
           		angles = np.radians(index)
			if len(lsr)> 0 and angles.shape[0] == len(lsr):
           			converted_scans = np.array([np.cos(angles), np.sin(angles)]).T * np.array(lsr)[:, np.newaxis]
          			lasers.append(np.array(converted_scans))
				lasersScan.append(lsr)
		if (numScan + 2) % 3 == 0:
			odom = dataLine.split(" ")
			odm = []
			for o in odom:
				if not o == "":
					odm.append(float(o))
			print(odm)
			odoms.append(odm)
		if (numScan+1)%3 == 0:
			
			imageHash.append(int(dataLine))
		numScan += 1
	
	return lasersScan,lasers,odoms,imageHash
