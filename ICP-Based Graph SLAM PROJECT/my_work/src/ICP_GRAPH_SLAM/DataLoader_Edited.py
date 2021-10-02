import numpy as np
import math
from sensor_msgs.msg import LaserScan

def RangeToPCL(laserScan):
	angleInc = laserScan.angle_increment
	scanAngle = laserScan.angle_min
	
	points = []

	laserScn = np.array(laserScan.ranges)
	laserScn[laserScn == np.inf] =0

	for scanRange in laserScn:
		if scanRange > 0.05:

			point_x = scanRange * math.cos(scanAngle)
			point_y = scanRange * math.sin(scanAngle)
			point = np.array([point_x,point_y]) #converted to 2d
			points.append(point)
		
		scanAngle += angleInc
		
	return np.array(points)



