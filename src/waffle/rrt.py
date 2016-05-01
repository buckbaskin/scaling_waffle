# pylint: disable=line-too-long
# pylint: disable=invalid-name

import math
import random

from collections import deque
from copy import deepcopy
from geometry_msgs.msg import Pose
from math import sin, cos
# from sensor_msgs.msg import LaserScan
from waffle.waffle_common import Planner, ROBOT_RADIUS
from utils import quaternion_to_heading, heading_to_quaternion, addv


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
                                'center': ObstacleMap(self.minx/2.0+self.midx/2.0, self.maxx/2.0+self.midx/2.0,
                                    self.midy, self.maxy),
                                'right': ObstacleMap(self.midx, self.maxx,
                                    self.midy, self.maxy)},
                        'middle':{'left':ObstacleMap(self.minx, self.midx,
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0),
                                  'center': ObstacleMap(self.minx/2.0+self.midx/2.0, self.maxx/2.0+self.midx/2.0,
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0),
                                  'right': ObstacleMap(self.midx, self.maxx,
                                    self.miny/2.0+self.midy/2.0, self.maxy/2.0+self.midy/2.0)},
                        'bottom':{'left':ObstacleMap(self.minx, self.midx,
                                    self.miny, self.midy),
                                  'center': ObstacleMap(self.minx/2.0+self.midx/2.0, self.maxx/2.0+self.midx/2.0,
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


class RRTNode(Pose):
    def __init__(self, pose):
        super(RRTNode, self).__init__()
        self.position.x = pose.position.x
        self.position.y = pose.position.y
        self.position.z = pose.position.z
        self.orientation.w = pose.orientation.w
        self.orientation.x = pose.orientation.x
        self.orientation.y = pose.orientation.y
        self.orientation.z = pose.orientation.z
        
        self.rrt_parent = None # this is for the RRT, positional children
        self.rrt_children = []

        self.kd_parent = None # this is for the kd tree, positional children
        self.kd_left = None # this is for the kd tree, positional children
        self.kd_right = None # this is for the kd tree, positional children

class RRT(dict):
    def __init__(self):
        super(RRT, self).__init__()

        self.obstacles = ObstacleMap(-5, 45, -5, 45)
        self.goal = None

        # start self at odometry 0
        self[0] = RRTNode(Pose())

        self.next_id = 1

    def add_node_kd(self, rrt_node_id, depth=0, compare_id=0):
        '''
        The no code repeating version of adding a node to a kd tree
        '''
        if depth % 2 == 0:
            feature = 'x'
        else:
            feature = 'y'

        if getattr(self[rrt_node_id], feature) < getattr(self[compare_id], feature):
            side = 'left'
        else:
            side = 'right'

        if getattr(self[compare_id], side) is None:
            # if there is no child for the given node on this side
            setattr(self[compare_id], side, rrt_node_id):
            return
        else:
            # if there is a child on this side, recursively call...
            self.add_node_kd(rrt_node_id, depth+1, get(self[compare_id], side))

    def add_node_rrt(self, pose):
        # find its closest neighbor by id
        # set that to be its parent
        self[self.next_id] = RRTNode(pose)
        self[self.next_id].rrt_parent = self.find_nearest_node(pose)
        self[self.next_id].rrt_children = []
        self.add_node_kd(self.next_id)
        self.next_id += 1
        return self.next_id - 1

    def distance_function(self, pose1, pose2):
        # TODO(buckbaskin):
        pass

    def expand_tree(self):
        if self.reached_goal():
            # don't expand
            return
        if self.goal is not None and random.uniform(0.0, 1.0) < .10:
            # choose the goal with 10% certainty
            expand_to_pose = self.goal
        else:
            # put the expand_to_pose in random free space
            x = random.uniform(self.minx, self.maxx)
            y = random.uniform(self.miny, self.maxy)

            expand_to_pose = Pose()
            expand_to_pose.position.x = x
            expand_to_pose.position.y = y

            while self.obstacles.check_collision(expand_to_pose):
                x = random.uniform(self.minx, self.maxx)
                y = random.uniform(self.miny, self.maxy)

                expand_to_pose.position.x = x
                expand_to_pose.position.y = y

        expand_to_pose = RRTNode()

        # try to expand up to that node, storing the expand_from node
        expand_from_id = self.find_nearest_node(node)
        expand_from_pose = self[expand_from_id]

        collision_step = 0.1
        plan_step = 1.0

        collision_steps_per_plan = int(plan_step/collision_step)

        count = 1

        dx = expand_to_pose.position.x - expand_from_pose.position.x
        dy = expand_to_pose.position.y - expand_from_pose.position.y

        distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

        collision_steps = int(distance/collision_step)

        dx = (dx / distance) * collision_step
        dy = (dy / distance) * collision_step

        collision_pose = Pose()
        collision_pose.position.x = expand_from_pose.position.x + dx
        collision_pose.position.y = expand_from_pose.position.y + dy

        # check collision
        # self.obstacles.check_collision(choose_pose)

        #   if that node is less than one step away, check collision and add it
        #   else, maintain the last step and second to last step.
        #       if there is a collision in the last step, add the second to last
        #           as an element in the rrt
        #   otherwise, loop until the first step reaches the goal, see first if
        while count <= collision_steps:
            if self.obstacles.check_collision(collision_pose):
                # if there was a collision
                if not (count % collision_steps_per_plan) == 1:
                    # if I've advanced past the first node after a plan step
                    # step back
                    collision_pose.position.x += -dx
                    collision_pose.position.y += -dy
                    # make a new node at that point
                    self.add_node_rrt(collision_pose)
                    break
                else:
                    # I just added a plan-step node
                    break
            else:
                # there isn't an obstacle, I can keep going
                if (count % collision_steps_per_plan) == 0:
                    # add a planner-step pose (every meter)
                    self.add_node_rrt(collision_pose)

                collision_pose.position.x += dx
                collision_pose.position.y += dy
                distance += -collision_step
                count += 1

        if distance <= collision_step:
            # the loop ran all the way through
            if not self.obstacles.check_collision(expand_to_pose):
                # if there isn't a collision at the expand_to pose
                self.add_node_rrt(expand_to_pose)

    
    def find_nearest_node(self, pose):
        # TODO(buckbaskin):
        pass

    def find_nearest_node_down(self, pose, depth=0, root_index=0):
        # TODO(buckbaskin):
        pass

    def find_nearest_node_up(self, pose, depth):
        # TODO(buckbaskin):
        pass

    def reached_goal(self):
        # TODO(buckbaskin):
        # check if the nearest pose is less than one collision step
        pass

    def remove_node_by_id(self, pose):
        # TODO(buckbaskin):
        # remove child id from rrt parent
        # remove children recursively
        # once I have no children/am a rrt leaf node
        #   remove myself from my kd parent
        #   add all remaining kd-children back to the kd tree
        pass