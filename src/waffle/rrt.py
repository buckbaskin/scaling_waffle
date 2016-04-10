from collections import deque
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from waffle_common import Planner

robot_radius = .5

class ObstacleMap(object):
    def __init__(self, minx, maxx, miny, maxy):
        self.children = None
        self.obstacle_list = []
        self.minx = minx
        self.maxx = maxx
        self.midx = minx/2+maxx/2
        self.miny = miny
        self.maxy = maxy
        self.midy = miny/2+maxy/2

    def distance_function(self, pose1, pose2):
        return math.sqrt(math.pow(pose1.position.x-pose2.position.x, 2) +
            math.pow(pose1.position.y-pose2.position.y, 2) +
            math.pow(pose1.position.z-pose2.position.z, 2))

    def add_obstacle(self, pose, radius):
        if pose.position.x > self.maxx:
            return None
        if pose.position.y > self.maxy:
            return None
        if pose.position.x < self.minx:
            return None
        if pose.position.y < self.miny:
            return None
        if children = None:
            # then there are no sub-maps
            self.obstacle_list.append((pose, radius,))
            if len(self.obstacle_list) > 20:
                self.sub_divide()
        else:
            for vertical in ['top', 'middle', 'bottom']:
                for horizontal in ['left', 'center', 'right']:
                    children[vertical][horizontal].add_obstacle(pose, radius)

    def sub_divide(self):
        self.children = {'top':{'left':ObstacleMap(self.minx, self.midx,
                                    self.midy, self.maxy),
                                'center': ObstacleMap(self.minx/2.0+self.midx/2.0,
                                    self.midy, self.maxy),
                                'right': ObstacleMap(self.midx, self.maxx,
                                    self.midy, self.maxy)},
                        'middle':{'left':ObstacleMap(self.minx, self.midx,
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0),
                                  'center': ObstacleMap(self.minx/2.0+self.midx/2.0,
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0),
                                  'right': ObstacleMap(self.midx, self.maxx,
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0)}
                        'bottom':{'left':ObstacleMap(self.minx, self.midx,
                                    self.miny, self.midy),
                                  'center': ObstacleMap(self.minx/2.0+self.midx/2.0,
                                    self.miny, self.midy),
                                  'right': ObstacleMap(self.midx, self.maxx,
                                    self.miny, self.midy)}}
        for obstacle in obstacle_list:
            # this will add it to the children
            self.add_obstacle(obstacle[0], obstacle[1])
        self.obstacle_list = None


    def check_collision(self, pose):
        if children is None:
            for obstacle in obstacle_list:
                obstacle_pose = obstacle[0]
                obstacle_radius = obstacle[1]
                if self.distance_function(pose, obstacle_pose) < obstacle_radius + robot_radius:
                    return True
        else:
            for vertical in ['top', 'middle', 'bottom']:
                for horizontal in ['left', 'center', 'right']:
                    if children[vertical][horizontal].check_collision(pose):
                        return True
        return False

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

        - Generate new obstacles (every scan is a small obstacle)
        - For each new obstacle, find the nearest tree node. If the tree node is
        in the obstacle, prune it. Repeat until there are no nodes to prune for
        that obstacle

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

