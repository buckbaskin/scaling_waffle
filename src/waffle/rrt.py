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


class SpecialPose(Pose):
    def __init__(self, pose):
        super(SpecialPose, self).__init__()
        self.position.x = pose.position.x
        self.position.y = pose.position.y
        self.position.z = pose.position.z
        self.orientation.w = pose.orientation.w
        self.orientation.x = pose.orientation.x
        self.orientation.y = pose.orientation.y
        self.orientation.z = pose.orientation.z
        self.parent = None # this is for the RRT, positional children
        self.children = []
        self.kd_parent = None # this is for the kd tree, positional children
        self.left = None # this is for the kd tree, positional children
        self.right = None # this is for the kd tree, positional children

class NodeTree(dict):
    '''
    a kd-tree for finding the nearest nodes to a given point
    '''
    def __init__(self, start_pose):
        super(NodeTree, self).__init__()
        self[0] = SpecialPose(start_pose)
        self[0].parent = None
        self[0].left = None # this is for the kd tree, positional children
        self[0].right = None # this is for the kd tree, positional children
        self.next_id = 1

    def distance_function(self, pose1, pose2):
        return math.sqrt(math.pow(pose1.position.x - pose2.position.x, 2)+
            math.pow(pose1.position.y - pose2.position.y, 2))

    def find_nearest_node(self, test_pose):
        node_id, best_distance, depth = self.find_nearest_node_down(test_pose)
        node_id = self.find_nearest_node_up(test_pose, node_id, best_distance, node_id, depth)
        return node_id

    def find_nearest_node_down(self, test_pose, depth=0, root_index=0):
        if depth % 2 == 0:
            # check x
            if test_pose.position.x < self[root_index].position.x:
                # left
                if self[root_index].left is None:
                    # leaf
                    return (root_index, self.distance_function(test_pose, self[root_index]), depth,)
                else:
                    # follow child down the rabbit hole
                    return self.find_nearest_node_down(test_pose, depth+1, self[root_index].left)
            else:
                # right
                if self[root_index].right is None:
                    # leaf
                    return (root_index, self.distance_function(test_pose, self[root_index]), depth,)
                else:
                    # follow child down the rabbit hole
                    return self.find_nearest_node_down(test_pose, depth+1, self[root_index].right)
        else:
            # check y
            if test_pose.position.y < self[root_index].position.y:
                # left
                if self[root_index].left is None:
                    # leaf
                    return (root_index, self.distance_function(test_pose, self[root_index]), depth,)
                else:
                    # follow child down the rabbit hole
                    return self.find_nearest_node_down(test_pose, depth+1, self[root_index].left)
            else:
                # right
                if self[root_index].right is None:
                    # leaf
                    return (root_index, self.distance_function(test_pose, self[root_index]), depth,)
                else:
                    # follow child down the rabbit hole
                    return self.find_nearest_node_down(test_pose, depth+1, self[root_index].right)
        return (0, float('inf'), depth,)

    def find_nearest_node_up(self, test_pose, best_node_id, best_distance, 
        pivot_index, depth):
        if depth < 0 or pivot_index is None:
            # end recursion
            return best_node_id
        if depth % 2 == 0:
            # check x
            if test_pose.position.x < self[pivot_index].position.x:
                # the test pose is left of the pivot that I'm checking
                if self[pivot_index].right is not None:
                    # there are nodes to the right of the pivot
                    crossover_distance = abs(test_pose.position.x-self[pivot_index].position.x)
                    if crossover_distance >= best_distance:
                        # check the pivot node, then check the others
                        distance_to_pivot = self.distance_function(self[pivot_index], test_pose)
                        if distance_to_pivot < best_distance:
                            best_distance = distance_to_pivot
                            best_node_id = pivot_index
                        # there may be any nodes on the other side that are closer
                        other_best_id, other_best_distance, other_depth = self.find_nearest_node_down(test_pose, depth, self[pivot_index].right)
                        if other_best_distance < best_distance:
                            # there is a closer node
                            return self.find_nearest_node_up(test_pose, other_best_id, other_best_distance, self[pivot_index].kd_parent, depth-1)
                # there are no nodes or no closer nodes to the right of the pivot
                # keep moving up
                return self.find_nearest_node_up(test_pose, best_node_id, best_distance, self[pivot_index].kd_parent, depth-1)
            else:
                # the test pose is right of the pivot that I'm checking
                if self[pivot_index].left is not None:
                    # there are nodes to the right of the pivot
                    crossover_distance = abs(test_pose.position.x-self[pivot_index].position.x)
                    if crossover_distance >= best_distance:
                        # there may be any nodes on the other side that are closer
                        # check the pivot node, then check the others
                        distance_to_pivot = self.distance_function(self[pivot_index], test_pose)
                        if distance_to_pivot < best_distance:
                            best_distance = distance_to_pivot
                            best_node_id = pivot_index

                        other_best_id, other_best_distance, other_depth = self.find_nearest_node_down(test_pose, depth, self[pivot_index].left)
                        if other_best_distance < best_distance:
                            # there is a closer node
                            return self.find_nearest_node_up(test_pose, other_best_id, other_best_distance, self[pivot_index].kd_parent, depth-1)
                # there are no nodes or no closer nodes to the right of the pivot
                # keep moving up
                return self.find_nearest_node_up(test_pose, best_node_id, best_distance, self[pivot_index].kd_parent, depth-1)
        else:
            # check y
            if test_pose.position.y < self[pivot_index].position.y:
                # left
                pass
            else:
                # right
                pass
        return 0

    def add_node(self, pose, depth=0, root_index=0):
        # add the node to a dict
        self[self.next_id] = pose
        self[self.next_id].left = None
        self[self.next_id].right = None
        # set the node's parent
        if depth % 2 == 0:
            # check x
            if pose.position.x < self[root_index].position.x:
                # left
                if self[root_index].left is None:
                    self[root_index].left = self.next_id
                    self[self.next_id].parent = root_index
                else:
                    self.add_node(pose, depth=depth+1, root_index=self[root_index].left)
            else:
                # right
                if self[root_index].right is None:
                    self[root_index].right = self.next_id
                    self[self.next_id].parent = root_index
                else:
                    self.add_node(pose, depth=depth+1, root_index=self[root_index].right)
        else:
            # check y
            if pose.position.y < self[root_index].position.y:
                # left
                if self[root_index].left is None:
                    self[root_index].left = self.next_id
                    self[self.next_id].parent = root_index
                else:
                    self.add_node(pose, depth=depth+1, root_index=self[root_index].left)
            else:
                # right
                if self[root_index].right is None:
                    self[root_index].right = self.next_id
                    self[self.next_id].parent = root_index
                else:
                    self.add_node(pose, depth=depth+1, root_index=self[root_index].right)
        # increment the node's next insert index
        self.next_id += 1
        # return the id of the inserted node
        return self.next_id - 1

class RRTBase(Planner):
    def __init__(self, minx=-100.0, maxx=100.0, miny=-100.0, maxy=100.0):
        super(RRTBase, self).__init__()
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.obstacles = ObstacleMap(minx, maxx, miny, maxy)

        self.step_size = .1

        self.nodes = NodeTree(Pose())

        self.start = Pose()
        self.goal = None

    def set_root(self, pose):
        self.nodes = NodeTree(pose)

    def set_goal(self, pose):
        self.goal = pose

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

            self.obstacles.add_obstacle(deepcopy(new_pose), radius)

            # check if I need to prune
            self.prune_tree(new_pose, radius)

            angle += scan.angle_increment

        # remove previously seen obstacles that are too close to existing
        #   obstacles

        self.obstacles.condense()

    def add_node(self, pose):
        rrt_parent = self.find_nearest_node(pose)
        rrt_children = []
        location = pose
        sp = SpecialPose(location)
        sp.parent = rrt_parent
        sp.children = rrt_parent
        self.nodes.add_node(sp)

    def expand_tree_biased(self):
        if self.goal is None:
            self.expand_tree()
            return
        if random.uniform(0.0, 1.0) < .05:
            choose_pose = self.goal

            nearest_id = self.find_nearest_node(choose_pose)

            dx = self.goal.position.x - self.nodes[nearest_id].position.x
            dy = self.goal.position.x - self.nodes[nearest_id].position.y

            dist = math.sqrt(dx*dx + dy*dy)
            dx = dx/dist*self.step_size
            dy = dy/dist*self.step_size

            if dist < .01:
                # found goal
                return None
            elif dist < self.step_size:
                if not self.obstacles.check_collision(self.goal):
                    self.add_node(self.goal)
                return None
            else:
                choose_pose.position.x = self.nodes[nearest_id].position.x + dx
                choose_pose.position.y = self.nodes[nearest_id].position.y + dy
                # I need to take steps towards the goal.
                # TODO(buckbaskin): go until collision, only add the last one
                #   before collision
                while dist > self.step_size:
                    if self.obstacles.check_collision(choose_pose):
                        return None
                    else:
                        self.add_node(choose_pose)

                        choose_pose.position.x += dx
                        choose_pose.position.y += dy

                        dx = self.goal.position.x - choose_pose.position.x
                        dy = self.goal.position.y - choose_pose.position.x

                        dist = math.sqrt(dx*dx + dy*dy)
                        dx = dx/dist*self.step_size
                        dy = dy/dist*self.step_size
                if dist < self.step_size:
                    if self.obstacles.check_collision(self.goal):
                        return None
                    else:
                        self.add_node(self.goal)
                return None
        else:
            self.expand_tree()

    def expand_tree(self):
        '''
        Based on all past information, expand the tree
        '''

        x = random.uniform(self.minx, self.maxx)
        y = random.uniform(self.miny, self.maxy)

        choose_pose = Pose()
        choose_pose.position.x = x
        choose_pose.position.y = y

        while self.obstacles.check_collision(choose_pose):
            x = random.uniform(self.minx, self.maxx)
            y = random.uniform(self.miny, self.maxy)

            choose_pose = Pose()
            choose_pose.position.x = x
            choose_pose.position.y = y

        # now I have a point in the free space

        nearest_id = self.find_nearest_node(choose_pose)

        dx = choose_pose.position.x - self.nodes[nearest_id].position.x
        dy = choose_pose.position.x - self.nodes[nearest_id].position.y

        dist = math.sqrt(dx*dx + dy*dy)
        dx = dx/dist*self.step_size
        dy = dy/dist*self.step_size

        if dist < .01:
            return None # point already taken care of...
        elif dist < 2.0*self.step_size:
            # if the point is close, add the choose point as the child
            if not self.obstacles.check_collision(choose_pose):
                self.add_node(choose_pose)
        else:
            # TODO(buckbaskin): take steps until collision, then add the last one
            # take a step towards the chosen
            choose_pose.position.x = self.nodes[nearest_id].position.x + dx
            choose_pose.position.y = self.nodes[nearest_id].position.y + dy
            if not self.obstacles.check_collision(choose_pose):
                self.add_node(choose_pose)

    def prune_tree(self, pose, radius):
        '''
        Find the nearest node to the pose/radius. If it is within the radius
        /collision, then remove that node from the tree (and its children)

        Repeat until the nearest node isn't a collision
        '''
        nearest_id = self.find_nearest_node(pose)
        dx = pose.position.x - self.nodes[nearest_id].position.x
        dy = pose.position.y - self.nodes[nearest_id].position.y
        dist = math.sqrt(dx*dx + dy*dy)
        dist += -radius
        dist += -ROBOT_RADIUS

        while dist < 0:
            # collision!
            self.nodes = self.nodes[nearest_id].remove(self.nodes)

            nearest_id = self.find_nearest_node(pose)
            dx = pose.position.x - self.nodes[nearest_id].position.x
            dy = pose.position.y - self.nodes[nearest_id].position.y
            dist = math.sqrt(dx*dx + dy*dy)
            dist += -radius
            dist += -ROBOT_RADIUS

    def find_nearest_node(self, pose):
        '''
        kd-tree?
        Input:
            Pose
        Output
            int (node id)
        '''
        return self.nodes.find_nearest_node(pose)

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
