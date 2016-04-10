from collections import deque
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from waffle_common import Planner

class RRTNode(object):
    def __init__(self, pose, parent):
        self.pose = pose
        self.parent = int(parent)
        self.children = []


class RRTBase(Planner):
    def __init__(self):
        super(RRT, self).__init__()

        self.nodes = {}

    def set_root(self, pose):
        self.nodes = {}
        self.nodes[0] = RRTNode(Pose, None)

    def new_scan(self, pose, scan):
        '''
        based on a new set of scan data collected at a given pose, update
        obstacles, then update/prune the tree

        Input:
            Pose
            LaserScan
        Output:
            None
        '''
        pass

    def expand_tree(self):
        '''
        Based on all past information, expand the tree
        '''
        pass

    def find_nearest_node(self, pose):
        '''
        Input:
            Pose
        Output
            int (node id)
        '''
        return 0

    def find_path(self, start_id, goal_id):
        '''
        Find a path through the tree from the start id to the goal id
        Input:
            int
            int
        Output:
            list of int
        '''
        return [0]

    def smooth_path(self, path):
        '''
        For a given path, try to cut corners and find an easier path than just
        following the RRT
        
        Input:
            list of int
        Output:
            list of int
        '''
        return path

    def generate_plan(self, start, goal):
        '''
        Based on the current tree, generate a plan for reaching the goal pose
        '''
        return deque()

