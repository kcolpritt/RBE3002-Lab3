
#Alex Krasa 
#RBE 3002
#Lab #3
#PreLab

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from AStar import AStar

def _init_(self, variable):

    pass

def MapCallback(event):
    pass

if __name__ == '__main__':
    rospy.init_node('sample_Pre_Lab_3_node_ajkrasa')
   
    global pub
    global pose
    global odom_tf
    global odom_list

    MapSub = rospy.Subscriber('/map', OccupancyGrid, readWorldMap)
    MarkerSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal)
    sub = rospy.Subscriber("/initialPose", PoseWithCovarianceStamped, startCallBack)

    odom_list = tf.TransformListener()

    rospy.sleep(rospy.Duration(1, 0))

    print "Pre_Lab 3 complete!"

