#!/usr/bin/env python

import rospy, tf, numpy, math
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
from Queue import PriorityQueue

#Our A* Code
from astar



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

	#value - the cost associated getting to this gridcell (int)
	#coor - coordinates of the current gridcell (tuple)
	#start- coordinates of the starting gridcell (tuple)
	#goal- coordinates of the goal (tuple)
        #parent- previous gridcell (Grid)
    def _init_(self, parent, coor, value, start, goal):
        self.value = value
        self.children = []
        self.parent = parent
        self.coor = coor
        self.dist = 0
        if parent:
            self.path = parent.path[:]
            self.path.append(coor)
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
    def _init_(self, value, parent, start, goal, coor):
        super(Grid_String, self) ._init_(value, parent, start, goal)
        self.dist = self.GetDist()
    def GetDist(self):
            if(self.value == self.goal):
                return 0
            dist=0
            x_range = self.goal[0]
            y_range = self.goal[1]
            dist += sqrt(x_range*x_range + y_range*y_range)
            return dist

    #needs to be modified
    def CreateChildren(self):
        if not (self.children):
            for i in xrange(len(self.goal)-1):
                val = self.value
                val = value + 1
                child = Grid_String(val, self)
                self.children.append(child)
          

# The xternal file might work better, but it needs testing and implimentation
class AStar_Solver:
    def _init_(self, start, goal):
        self.path = []
        self.visitedQueue = []
        self.priorityQueue = PriorityQueue()
        self.start = start
        self.goal = goal

    def Solve(self):
        startGrid = Grid_String(0, 0, self.start, self.goal, self.start)
        count = 0
        self.priorityQueue.put((0, count, startGrid))
        while(not self.path and self.priorityQueue.qsize()):
            closestChild = self.priorityQueue.get()[2]
            closestChild =.CreatChildren()
            self.visitedQueue.append(closestChild, value)
            for child in closestChild.children:
                if child.value not in self.visitedQueue:
                    count += 1
                    if not child.dist:
                        self.path = child.path
                        break
                    self.priorityQueue.put((child.dist, count, child))
        if not (self.path):
            print "Goal of" + self.goal + "is not possible"
        return self.path


def aStar(start,goal):
    pass
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points


closedset = the empty set    # The set of nodes already evaluated.
    openset = [start]            # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed 
                                 # set and all other nodes.
    came_from = the empty map    # The map of navigated nodes.
    
    # The g_score of a node is the distance of the shortest path from the start to the node.
    # Start by assuming that all nodes that have yet to be processed cannot be reached 
    g_score = map with default value of Infinity
    
    # The starting node has zero distance from start
    g_score[start] = 0
    
    # The f_score of a node is the estimated total cost from start to goal to the goal.  This is the sum of the g_score (shortest known path) and the h_score (best possible path).
    # assume same as g_score
    f_score = map with default value of Infinity  
    
    # heuristic_cost_estimate(a, b) is the shortest possible path between a and b, this can be euclidean, octodirectional, Manhattan, or something fancy based on how the machine moves
    # the best possible distance between the start and the goal will be the heuristic
    f_score[start] = g_score[start] + heuristic_cost_estimate(start, goal)
     
    
    while openset is not empty                                          # while there are still nodes that have not been checked, continually run the algorithm
    
        current = the node in openset having the lowest f_score[] value # this is the most promising node of all nodes in the open set
        if current = goal                                               # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            return reconstruct_path(came_from, goal)
         
        remove current from openset                  # mark this node as having been evaluated
        add current to closedset 
        for each neighbor in neighbor_nodes(current) # re-evaluate each neighboring node
            if neighbor in closedset
                continue
            tentative_g_score = g_score[current] + dist_between(current,neighbor) # create a new g_score for the current neighbor by adding the g_score from the current node and
                                                                                  # the distance to the neighbor
 
            if neighbor not in openset or tentative_g_score < g_score[neighbor]                 # if the neighbor has not been evaluated yet, or if a better path to the neighbor has been found,
                                                                                                # update the neighbor
                came_from[neighbor] = current                                                   # The node to reach this node from in the best time is the current node
                g_score[neighbor] = tentative_g_score                                           # The G score of the node is what we tentatively calculated earlier
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal) # The F score is the G score and the heuristic
                if neighbor not in openset                                                      # add this neighbor to the frontier if it was not in it already
                    add neighbor to openset
 
    return failure #if the program runs out of nodes to check before it finds the goal, then a solution does not exist



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

   for i in range(1,height+1): #height should be set to hieght of grid
        for j in range(1,width+1): #width should be set to width of grid
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution) + offsetX - (.5 * resolution) # added secondary offset 
                point.y=(i*resolution) + offsetY - (.5 * resolution) # added secondary offset 
                point.z=0
                cells.cells.append(point)
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
