#!/usr/bin/env python
import rospy
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import quaternion_from_euler
import tf
import numpy
import math
import rospy, tf, numpy, math

class Node:
	def __init__(self,value, coor, start, goal, grid):
		self.value = value
		self.coor = coor
		self.parent = []
		self.start = start
		self.goal = goal
		self.children = []
		self.grid = grid
		self.path = []
		if(len(self.path) == 0):
			self.path.append(start)
		
	def Children(self):
		x = self.coor[0]
		y = self.coor[1]
		v = self.value
		s = self.start
		g = self.goal
		m = self.grid
		width = self.grid.info.width
		height = self.grid.info.height

		for i in range(0, height): #height should be set to hieght of grid
			for j in range(0, width): #width should be set to width of grid
				#if a gridcell is occupied no child is made in that gridcell
				if (self.grid.data[i*width+j] == 100):
					pass
				elif (self.grid.data[(i+1)*width+j] == 100):
					pass
				elif (self.grid.data[i*width+(j+1)] == 100):
					pass
				elif (self.grid.data[(i-1)*width+j] == 100):
					pass
				elif (self.grid.data[i*width+(j-1)] == 100):
					pass
				elif(x+1 == i and y == j):
					child1 = Node(v+1, (x+1, y), s, g, m)
					self.children.append(child1)
				elif(x == i and y+1 == j):
					child2 = Node(v+1, (x, y+1), s, g, m)
					self.children.append(child2)
				elif(x-1 == i and y == j):
					child3 = Node(v+1, (x-1, y), s, g, m)
					self.children.append(child3)
				elif(x == i and y-1 == j):
					child4 = Node(v+1, (x, y-1), s, g, m)
					self.children.append(child4)
				else:
					pass
		#print len(self.children)
						

	def Manhattan(self, point, point2):
		return abs(point[0] - point2[0]) + abs(point[1]-point2[1])
	
	def Greedy(self):
		x_side = abs(int(self.goal[0] - self.coor[0]))
		y_side = abs(int(self.goal[1] - self.coor[1]))
		hn = math.sqrt(pow(x_side, 2) + pow(y_side, 2))
		return hn

	def CalF(self):
		g = self.value
		h = self.Greedy()
		return g + h
		


def Solve(start, goal, node):	
	shortest = 0
	current = node
	child = None
	closedset = []
	#while(current.coor[0] != goal[0] and current.coor[1] != goal[1]):
	while(current.coor != goal):
		#print closedset
		current.Children()
		#if(len(closedset) != 0):
		#	for i in current.children:	
		#		for l in closedset:
		#			#print i.coor
		#			#rospy.sleep(1)
		#			if(i.coor == l):
		#				pass
		#			else:
		#				if(shortest == 0):
		#					child = i
		#					shortest = i.CalF()
		#				elif(i.CalF() < shortest):
		#					child = i
		#					shortest = i.CalF()
		#else:
		for i in current.children:
			print i.coor	
			if(shortest == 0):
				child = i
				shortest = i.CalF()
			elif(i.CalF() < shortest):
				child = i
				shortest = i.CalF()
		if(child != None):
			child.parent.append(current)
			node.path.append(current.coor)
			closedset.append(current.coor)
			current = child
			child = 0
			rospy.sleep(1)
			print start, current.coor, goal
			#print current.coor == goal
		shortest = 0
	node.path.append(current.coor)
	return node.path



def aStar(start, goal, grid):
	
	init = (start[0], start[1])
	end = (goal[0], goal[1])

	
	origin = Node(0, init, init, end, grid)
	solution = Solve(init, end, origin)
	#print solution
	return solution
	
