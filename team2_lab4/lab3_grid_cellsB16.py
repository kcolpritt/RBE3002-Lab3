#!/usr/bin/env python

import rospy
from astar import aStar
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
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







def Waypoints():

	#waypointpub = rospy.Publisher('Waypoint_Cells', GridCells)
	#WaypointCells = GridCells()
	#WaypointCells.cell_width = 1
	#WaypointCells.cell_height = 1
	#WaypointCells.cells = [Point()]
	#WaypointCells.header.frame_id = 'map'
	way = []
	i = 0
	pointList = aStar((3, 3), (7, 5), mapGrid)
	navPath = Path()
	pathPose = PoseStamped()	

	for item in pointList:
		current_x = item[0]
		current_y = item[1]
		next_point = pointList[i+1]
		next_x = next_point[0]
		next_y = next_point[1]
		if(next_x == current_x+1 and next_y == current_y):
			if(i == 0 or i == 1):
				i = 1
			else:
				i = 1
				way.append((current_x, current_y, 0))
				pathPose.pose.position.x = current_x
				pathPose.pose.position.y = current_y
				pathPose.pose.position.z = 0
				yaw = math.radians(0)
				roll = 0 
				pitch = 0
				quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
				pathPose.pose.orientation.x = quaternion[0]
				pathPose.pose.orientation.y = quaternion[1]
				pathPose.pose.orientation.z = quaternion[2]
				pathPose.pose.orientation.w = quaternion[3]

		elif(next_x == current_x and next_y == current_y+1):
			if(i == 0 or i == 2):
				i = 2
			else:
				i = 2
				way.append((current_x, current_y, 90))
				pathPose.pose.position.x = current_x
				pathPose.pose.position.y = current_y
				pathPose.pose.position.z = 0
				yaw = math.radians(0)
				roll = 0 
				pitch = 0
				quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
				pathPose.pose.orientation.x = quaternion[0]
				pathPose.pose.orientation.y = quaternion[1]
				pathPose.pose.orientation.z = quaternion[2]
				pathPose.pose.orientation.w = quaternion[3]

		if(next_x == current_x-1 and next_y == current_y):
			if(i == 0 or i == 3):
				i = 3
			else:
				i = 3
				way.append((current_x, current_y, 180))
				pathPose.pose.position.x = current_x
				pathPose.pose.position.y = current_y
				pathPose.pose.position.z = 0
				yaw = math.radians(0)
				roll = 0 
				pitch = 0
				quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
				pathPose.pose.orientation.x = quaternion[0]
				pathPose.pose.orientation.y = quaternion[1]
				pathPose.pose.orientation.z = quaternion[2]
				pathPose.pose.orientation.w = quaternion[3]

		elif(next_x == current_x and next_y == current_y-1):
			if(i == 0 or i == 4):
				i = 4
			else:
				i = 4
				way.append((current_x, current_y, 270))
				pathPose.pose.position.x = current_x
				pathPose.pose.position.y = current_y
				pathPose.pose.position.z = 0
				yaw = math.radians(0)
				roll = 0 
				pitch = 0
				quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
				pathPose.pose.orientation.x = quaternion[0]
				pathPose.pose.orientation.y = quaternion[1]
				pathPose.pose.orientation.z = quaternion[2]
				pathPose.pose.orientation.w = quaternion[3]
		
		navPath.poses.append(pathPose)

	print way
	dispathpub.publish(navPath)





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
	dispathpub = rospy.Publisher("/BS_topic", Path, queue_size=1)

	# wait a second for publisher, subscribers, and TF
	rospy.sleep(1)


	while not rospy.is_shutdown():
		publishCells(mapData) #publishing map data every 2 seconds
		try:
			print "initializing"
			Waypoints()
			#retVal = aStar((3, 3), (7, 5), mapGrid)
			#if(retVal != None):
				#print "Publishing path"
		#		pubpath.publish(retVal)
			
		except NameError:
			rospy.sleep(.1)
		rospy.sleep(2)

	
	


if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
