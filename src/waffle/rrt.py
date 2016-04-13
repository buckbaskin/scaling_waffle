# pylint: disable=line-too-long

import math

from collections import deque
from geometry_msgs.msg import Pose
# from sensor_msgs.msg import LaserScan
from waffle.waffle_common import Planner, ROBOT_RADIUS

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
        if self.children is None:
            # then there are no sub-maps
            self.obstacle_list.append((pose, radius,))
            if len(self.obstacle_list) > 20:
                self.sub_divide()
        else:
            for vertical in ['top', 'middle', 'bottom']:
                for horizontal in ['left', 'center', 'right']:
                    self.children[vertical][horizontal].add_obstacle(pose, radius)

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
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0)},
                        'bottom':{'left':ObstacleMap(self.minx, self.midx,
                                    self.miny, self.midy),
                                  'center': ObstacleMap(self.minx/2.0+self.midx/2.0,
                                    self.miny, self.midy),
                                  'right': ObstacleMap(self.midx, self.maxx,
                                    self.miny, self.midy)}}
        for obstacle in self.obstacle_list:
            # this will add it to self.children
            self.add_obstacle(obstacle[0], obstacle[1])
        self.obstacle_list = None


    def check_collision(self, pose):
        if pose.position.x > self.maxx:
            return False
        if pose.position.y > self.maxy:
            return False
        if pose.position.x < self.minx:
            return False
        if pose.position.y < self.miny:
            return False

        if self.children is None:
            for obstacle in self.obstacle_list:
                obstacle_pose = obstacle[0]
                obstacle_radius = obstacle[1]
                if self.distance_function(pose, obstacle_pose) < obstacle_radius + ROBOT_RADIUS:
                    return True
        else:
            for vertical in ['top', 'middle', 'bottom']:
                for horizontal in ['left', 'center', 'right']:
                    if self.children[vertical][horizontal].check_collision(pose):
                        return True
        return False

    def condense(self):
        '''
        Combine multiple obstacles in the same place into one
        '''
        pass

class RRTNode(object):
    def __init__(self, pose, parent):
        self.pose = pose
        self.parent = int(parent)
        self.children = []
        self.obstacles = ObstacleMap(-100.0, 100.0, -100.0, 100.0)

    def add_child(self, pose):
        self.children.append(pose)


class RRTBase(Planner):
    def __init__(self):
        super(RRTBase, self).__init__()

        self.nodes = {}

    def set_root(self, pose):
        self.nodes = {}
        self.nodes[0] = RRTNode(pose, None)

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
        # add in all the new obstacles
        angle = scan.min_angle+quaternion_to_heading(pose.orientation)
        for reading in scan.ranges:

            # add a new obstacle

            if reading > scan.range_max - .01:
                continue
            x = pose.position.x + reading*cos(angle)
            y = pose.position.x + reading*cos(angle)
            radius = reading*scan.angle_increment/2.0

            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y

            self.obstacles.add_obstacle(deep_copy(new_pose), radius)

            # check if I need to prune
            self.prune_tree(new_pose, radius)

            angle += scan.angle_increment

        # remove previously seen obstacles that are too close to existing
        #   obstacles

        self.obstacles.condense()

    def expand_tree(self):
        '''
        Based on all past information, expand the tree
        '''
        #TODO(buckbaskin): implement this
        pass

    def prune_tree(self, pose, radius):
        '''
        Find the nearest node to the pose/radius. If it is within the radius
        /collision, then remove that node from the tree (and its children)

        Repeat until the nearest node isn't a collision
        '''
        # TODO(buckbaskin):
        pass

    def find_nearest_node(self, pose):
        '''
        kd-tree?
        Input:
            Pose
        Output
            int (node id)
        '''
        #TODO(buckbaskin): implement this
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
        #TODO(buckbaskin): implement this
        # start = self.nodes[start_id]
        # goal = self.nodes[goal_id]
        return [start_id, 0, goal_id]

    def smooth_path(self, path):
        '''
        For a given path, try to cut corners and find an easier path than just
        following the RRT

        Input:
            list of int
        Output:
            list of int
        '''
        #TODO(buckbaskin): implement this
        return path

    def generate_plan(self, start, goal):
        '''
        Based on the current tree, generate a plan for reaching the goal pose
        '''
        #TODO(buckbaskin): implement this
        deck = deque()
        deck.append(start)
        deck.append(Pose())
        deck.append(goal)
        return deck
