#!/usr/bin/env python

import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

import astar from aStar

def Waypoints (pointlist):

	#waypointpub = rospy.Publisher('Waypoint_Cells', GridCells)
	#WaypointCells = GridCells()
	#WaypointCells.cell_width = 1
	#WaypointCells.cell_height = 1
	#WaypointCells.cells = [Point()]
	#WaypointCells.header.frame_id = 'map'

	i = 0
	PointList = pointlist
	print PointList

	for item in PointList:
		current_x = item.x
		current_y = item.y
		next_point = pointList[i+1]
		next_x = next_point.x
		next_y = next_point.y
		print "current x", current_x
		print "current y", current_y
		print "next x", next_x
		print "next Y", next_y

	#	if (next_x - current_x != 0):
	#		WaypointCells.cells[0].x = current_x
	#		WaypointCells.cells[0].y = current_y
	#		WaypointCells.cells[0].z = 0
	#		gridPub.publish(WaypointCells)
	#		i += 1
	#	elif (next_y - current_y != 0):
	#		WaypointCells.cells[0].x = current_x
	#		WaypointCells.cells[0].y = current_y
	#		WaypointCells.cells[0].z = 0
	#		gridPub.publish(WaypointCells)
	#		i += 1


if __name__ == '__main__':
	rospy.init_node('lab3_waypoint_node')

	rospy.Subscriber('path', GridCells, Waypoints)

	#test = [(0,0,0), (1,1,1), (2,2,2,) (3,3,3) (2,3,2)]

	print "starting Waypoints"
	#Waypoints(test)
	print "end Waypoints"