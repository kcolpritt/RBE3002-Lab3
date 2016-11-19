#!/usr/bin/env python

import rospy
from astar import aStar
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math





#####################      COMMUNICATION      #############################

# reads in global map
def mapCallBack(data):
	global mapGrid
	global mapData
	global width
	global height
	global mapgrid
	global resolution
	global offsetX
	global offsetY
	mapGrid = data
	resolution = data.info.resolution
	mapData = data.data
	width = data.info.width
	height = data.info.height
	offsetX = data.info.origin.position.x
	offsetY = data.info.origin.position.y
	print data.info

def readGoal(goal):
	global goalRead
	goalRead = True
	global goalX
	global goalY
	global goalPos
	global goalIndex
	goalPos = goal
	goalX= goal.pose.position.x
	goalY= goal.pose.position.y
	
	#goalIndex = getIndexFromWorldPoint(goalX,goalY)
	#print "Printing goal pose"
	#print goal.pose


def readStart(_startPos):
	global startRead
	startRead = True
	global startPosX
	global startPosY
	global startPos
	global startIndex
	
	startPos = _startPos
	startPosX = startPos.pose.pose.position.x
	startPosY = startPos.pose.pose.position.y
	
	#startIndex = getIndexFromWorldPoint(startPosX, startPosY)
	#print "Printing start pose"
	#print startPos.pose.pose
	#point = getWorldPointFromIndex(startIndex)
	#print "Calculated world position: %f, %f Index: %i" % (point.x, point.y, startIndex)



#publishes map to rviz using gridcells type

def publishCells(grid):
	global pub
	#print "publishing"

	# resolution and offset of the map
	k=-2
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for i in range(0,height): #height should be set to hieght of grid
		for j in range(0,width): #width should be set to width of grid
			#print k # used for debugging
			if (grid[i*width+j] == 100):
				point=Point()
				point.x=(j*resolution)+offsetX + (.5 * resolution)
				point.y=(i*resolution)+offsetY + (.5 * resolution)
				point.z=0
				cells.cells.append(point)
	pub.publish(cells)		   













#####################      MAIN SETUP      #############################
def run():
	global pub
	rospy.init_node('lab3')
	sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
	pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
	pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
	pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
	goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
	navgoal_sub = rospy.Subscriber('move_base_simple/2goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
	goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

	# wait a second for publisher, subscribers, and TF
	rospy.sleep(1)


	# wait a second for publisher, subscribers, and TF
	rospy.sleep(1)

	retVal = 0

	while not rospy.is_shutdown():
		publishCells(mapData) #publishing map data every 2 seconds

		try:
			retVal = aStar(startPos, goalPos, mapGrid)
			if(retVal != None):
				print "Publishing path"
				pubpath.publish(retVal)
			
		except NameError:
			rospy.sleep(.1)
		rospy.sleep(2)

	
	


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
