#!/usr/bin/env python

import rospy, tf, numpy, math
import rospy
import tf
import numpy
import math
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
from Queue import PriorityQueue





# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def readGoal(goal):
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print goal.pose
    # Start Astar


def readStart(startPos):

    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print startPos.pose.pose

class Grid(object):

	#value - the cost associated getting to this gridcell
	#coor - coordinates of the current gridcell
	#start- coordinates of the starting gridcell
	#goal- coordinates of the goal
        #parent- previous gridcell
    def _init_(self, parent, coor, value, start, goal):
        self.value = value
        self.children = []
        self.parent = parent
        self.coor = coor
        self.dist = 0
        if parent:
            self.path = parent.path[:]
            self.path.append(value)
            self.start = parent.start
            self.goal = parent.path
        else: 
            self.path = value
            self.start = start
            self.goal = goal

        def GetDist(self):
            pass

        def CreateChildren(self):
            pass
                
class  Grid_String(Grid):
    def _init_(self, parent, coor, value, start goal):
        super(Grid_String, self) ._init_(value, parent, start, goal)
        self.dist = self.GetDist()
    def GetDist(self):
            if(self.value == self.goal):
                return 0
            dist=0

        def CreateChildren(self):
            pass
          

def aStar(start,goal):
    pass
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points

#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

   for i in range(1,height): #height should be set to hieght of grid
        for j in range(1,width): #width should be set to width of grid
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution) + offsetX - (.5 * resolution) # added secondary offset 
                point.y=(i*resolution) + offsetY - (.5 * resolution) # added secondary offset 
                point.z=0
                cells.cells.append(point)
            k += 1 
        k += 1  
    pub.publish(cells)           

#Main handler of the project
def run():
    global pub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)



    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
