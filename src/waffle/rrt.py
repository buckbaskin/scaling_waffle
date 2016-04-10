from collections import deque
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from waffle_common import Planner

class RRT(Planner):
	def __init__(self):
        super(RRT, self).__init__()

    def new_scan(self, pose, scan):
        '''
        based on a new set of scan data collected at a given pose, update the tree
        Input:
            Pose
            LaserScan
        Output:
            None
        '''
        pass

    def generate_plan(self, start, goal):
    	'''
    	Based on the current tree, generate a plan for reaching the goal pose
    	'''
    	return deque()

