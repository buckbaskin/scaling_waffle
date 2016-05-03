# pylint: disable=line-too-long
# pylint: disable=invalid-name

import rospy

import math
import random

from collections import deque
from copy import deepcopy
from geometry_msgs.msg import Pose
from heapq import heappush, heappop
from math import cos, sin
# from sensor_msgs.msg import LaserScan
from scaling_waffle.srv import PlanResponse
from waffle.waffle_common import Planner, ROBOT_RADIUS
from utils import quaternion_to_heading, addv

rospy.loginfo('imported w.w_common Planner: %s' % type(Planner))
rospy.loginfo('imported utils      addv: %s' % type(addv))

class MapSquare(object):
    def __init__(self):
        self.deck = deque()

    def distance_function(self, pose1, pose2):
        return math.sqrt(math.pow(pose1.position.x-pose2.position.x, 2) +
            math.pow(pose1.position.y-pose2.position.y, 2) +
            math.pow(pose1.position.z-pose2.position.z, 2))

    def add_obstacle(self, pose, radius):
        # maintain a list of the 100 nearest obstacles
        self.deck.appendleft((pose, radius,))
        while len(self.deck) > 100:
            self.deck.pop()

    def check_collision(self, other_pose):
        for pose, radius in list(self.deck):
            distance = self.distance_function(pose, other_pose)
            distance = distance - radius - ROBOT_RADIUS
            if distance < 0:
                return True


class ObstacleMap(object):
    def __init__(self, minx, maxx, miny, maxy):
        self.children = None
        self.obstacle_list = []
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.step_size = 0.25

        x_dist = self.maxx - self.minx
        y_dist = self.maxy - self.miny

        self.map = [[None]*int(y_dist/self.step_size)]*int(x_dist/self.step_size)
        for xx in xrange(0, len(self.map)):
            for yy in xrange(0, len(self.map[xx])):
                self.map[xx][yy] = MapSquare()

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

        x_dist = pose.position.x - self.minx
        y_dist = pose.position.y - self.miny

        x_bucket = int(x_dist / self.step_size)
        y_bucket = int(y_dist / self.step_size)

        self.map[x_bucket][y_bucket].add_obstacle(pose, radius)

    def check_loading(self):
        accum = 0
        for line in self.map:
            for square in line:
                accum += len(square.deck)
        rospy.loginfo('loading: %d/%d' % (accum, 100*len(self.map)*len(self.map)))

    def check_collision(self, pose):
        if pose.position.x > self.maxx:
            return False
        if pose.position.y > self.maxy:
            return False
        if pose.position.x < self.minx:
            return False
        if pose.position.y < self.miny:
            return False

        x_dist = pose.position.x - self.minx
        y_dist = pose.position.y - self.miny

        x_bucket = int(x_dist / self.step_size)
        y_bucket = int(y_dist / self.step_size)

        # check +- 3 buckets around the pose

        for xx in xrange(x_bucket-3, x_bucket+3+1):
            if xx < 0:
                continue
            if xx >= len(self.map):
                break
            for yy in xrange(y_bucket-3, y_bucket+3+1):
                if yy < 0:
                    continue
                if yy >= len(self.map[xx]):
                    break
                try:
                    if self.map[xx][yy].check_collision(pose):
                        return True
                except IndexError as ie:
                    rospy.loginfo('x: %d/%d, y: %d/%d' % (xx, len(self.map), yy, len(self.map[xx])))
                    raise ie
        return False

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
    def __init__(self, minx, maxx, miny, maxy):
        super(RRT, self).__init__()

        self.obstacles = ObstacleMap(-5.0, 35.0, -5.0, 35.0)
        self.goal = None

        # start self at odometry 0
        self[0] = RRTNode(Pose())

        self.next_id = 1

        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy

        self.kd_max_depth = 0
        self.rrt_max_depth = 0

    def add_node_kd_it(self, rrt_node_id, depth=0, compare_id=0):
        # rospy.loginfo('add_node_kd_it left %s right %s' % (self[compare_id].kd_left, self[compare_id].kd_right,))
        while depth < 10000:
            if depth % 2 == 0:
                feature = 'x'
            else:
                feature = 'y'

            # if depth == 1:
            #     rospy.loginfo('%f vs. the other %f' % (getattr(self[rrt_node_id].position, feature), getattr(self[compare_id].position, feature)))
            if getattr(self[rrt_node_id].position, feature) < getattr(self[compare_id].position, feature):
                side = 'kd_left'
            else:
                # rospy.loginfo('R')
                side = 'kd_right'

            if getattr(self[compare_id], side) is None:
                # if there is no child for the given node on this side
                setattr(self[compare_id], side, rrt_node_id)
                self[rrt_node_id].kd_parent = compare_id
                if self.kd_max_depth < depth:
                    self.kd_max_depth = depth
                return
            else:
                # if there is a child on this side, iteratively advance...
                depth += 1
                compare_id = getattr(self[compare_id], side)

    def add_node_kd(self, rrt_node_id, depth=0, compare_id=0):
        '''
        The no code repeating version of adding a node to a kd tree
        '''
        if depth % 2 == 0:
            feature = 'x'
        else:
            feature = 'y'

        if getattr(self[rrt_node_id].position, feature) < getattr(self[compare_id].position, feature):
            side = 'kd_left'
        else:
            side = 'kd_right'

        if getattr(self[compare_id], side) is None:
            # if there is no child for the given node on this side
            setattr(self[compare_id], side, rrt_node_id)
            self[rrt_node_id].kd_parent = compare_id
            return
        else:
            # if there is a child on this side, recursively call...
            self.add_node_kd_it(rrt_node_id, depth+1, getattr(self[compare_id], side))

    def add_node_rrt(self, pose):
        # find its closest neighbor by id
        # set that to be its parent
        self[self.next_id] = RRTNode(pose)
        # rospy.loginfo('add_node_rrt fnn: %s' % pose)
        self[self.next_id].rrt_parent = self.find_nearest_node(pose)
        self[self.next_id].rrt_children = []
        self.add_node_kd_it(self.next_id)
        self.next_id += 1
        return self.next_id - 1

    def distance_function(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

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

        # try to expand up to that node, storing the expand_from node
        # rospy.loginfo('expand_tree fnn: %s' % expand_to_pose)
        expand_from_id = self.find_nearest_node(expand_to_pose)
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

    def find_nearest_node(self, pose, debug=False):
        if debug:
            rospy.loginfo('begin debug of find_nearest_node_down_it')
        best_id, best_distance, depth = self.find_nearest_node_down_it(pose, debug=debug)
        if best_id == -1:
            raise KeyError('Find nearest node had an error.')
        if debug:
            rospy.loginfo('reprere (%d, %f, %d,)' % (best_id, best_distance, depth,))
        return self.find_nearest_node_up(pose, best_id, depth, best_id, best_distance)

    def print_parent_chain(self, start_id):
        accum = 'print_parent_chain\n'
        while start_id is not None:
            accum += start_id+'\n'
            if start_id == self[start_id].kd_parent:
                rospy.loginfo('error, child is its own parent')
            start_id = self[start_id].kd_parent
        rospy.loginfo(accum)

    def find_nearest_node_down_it(self, pose, depth=0, root_index=0, debug=False):
        # current_node = self[root_index]
        if debug:
            rospy.loginfo('find_nearest_node_down_it\n%d %d %r %r' % (depth, root_index, depth < 10000, debug,))
        while depth < 10000:
            if (depth % 2) == 0:
                feature = 'x'
            else:
                feature = 'y'

            if getattr(pose.position, feature) < getattr(self[root_index].position, feature):
                side = 'kd_left'
            else:
                side = 'kd_right'

            if debug:
                rospy.loginfo('d: %d i: %d f: %s s: %s' % (depth, root_index, feature, side))
            
            if getattr(self[root_index], side) is None:
                if depth > self.kd_max_depth:
                    self.kd_max_depth = depth
                    # rospy.loginfo('kd_max_depth %d' % (self.kd_max_depth,))
                return (root_index, self.distance_function(self[root_index], pose), depth,)
            else:
                depth += 1
                if debug:
                    rospy.loginfo('old root: %d new root: %d' % (root_index, getattr(self[root_index], side),))
                root_index = getattr(self[root_index], side)
        rospy.loginfo('fnndi no value. %d %d\n%s' % (depth, root_index, pose,))
        self.print_parent_chain()
        return (-1, float('inf'), depth,)

    def find_nearest_node_down(self, pose, depth=0, root_index=0):
        if (depth % 2) == 0:
            feature = 'x'
        else:
            feature = 'y'

        if getattr(pose.position, feature) < getattr(self[root_index].position, feature):
            side = 'kd_left'
        else:
            side = 'kd_right'

        if getattr(self[root_index], side) is None:
            if depth > self.kd_max_depth:
                self.kd_max_depth = depth
                # rospy.loginfo('kd_max_depth %d' % (self.kd_max_depth,))
            return (root_index, self.distance_function(self[root_index], pose), depth,)
        else:
            try:
                return self.find_nearest_node_down(pose, depth+1, getattr(self[root_index], side))
            except RuntimeError as rte:
                rospy.loginfo('probable max depth: %d \n%s' % (depth, rte,))
                raise rte


    def find_nearest_node_up(self, pose, next_id, depth, best_id, best_distance):
        if depth < 0 or next_id is None: # gone too far
            return best_id
        if depth == 0 or self[next_id].kd_parent is None: # stop at top of tree
            return best_id

        parent_id = self[next_id].kd_parent
        parent_depth = depth - 1

        if (parent_depth % 2) == 0:
            feature = 'x'
        else:
            feature = 'y'

        axial_distance = getattr(pose.position, feature) - getattr(self[parent_id].position, feature)

        if abs(axial_distance) < best_distance:
            # there could be a better node on the axis or on the other side
            parent_distance = self.distance_function(pose, self[parent_id])
            if parent_distance < best_distance:
                best_id = parent_id
                best_distance = parent_distance

            if axial_distance < 0:
                side = 'kd_left'
            else:
                side = 'kd_right'

            other_id, other_distance, dummy_depth = self.find_nearest_node_down(pose, parent_depth+1, getattr(self[parent_id], side))
            if other_distance < best_distance:
                best_id = other_id
                best_distance = other_distance
        
        # now I for sure have the best node 
        return self.find_nearest_node_up(pose, self[next_id].kd_parent, depth-1, best_id, best_distance)

    def generate_plan(self):
        end_id = 0
        if self.reached_goal():
            rospy.loginfo('gen plan: reached goal, doing A*')
            nodeheap = []
            heappush(nodeheap, (self.distance_function(self[0], self.goal), 0))
            
            while(len(nodeheap) > 0):
                heuristic, next_node_id = heappop(nodeheap)
                # I know this condition will be true because self.reached_goal()
                if self.distance_function(self[next_node_id], self.goal) < 0.1:
                    end_id = next_node_id
                    break
                else:
                    next_node_distance = heuristic - self.distance_function(self[next_node_id], self.goal)
                    
                    for child_id in self[next_node_id].rrt_children:
                        child_distance = next_node_distance + self.distance_function(self[child_id], self[next_node_id])
                        child_heuristic = self.distance_function(self[child_id], self.goal) + child_distance
                        heappush(nodeheap, (child_heuristic, child_id))

        else:
            if self.goal is None:
                rospy.loginfo('no goal yet :(')
                pr = PlanResponse()
                pr.allpoints = [Pose()]
                for val1 in pr.allpoints:
                    _x = val1.position
                    print('_x: %s' % _x)
                return pr.allpoints
            end_id = self.find_nearest_node(self.goal, True)

        deck = deque()
        deck.appendleft(self[end_id])
        while self[end_id].rrt_parent is not None:
            end_id = self[end_id].rrt_parent
            deck.appendleft(self[end_id])

        final_list = list(deck)
        if len(final_list) > 10:
            final_list = final_list[0:10]

        rospy.loginfo('legit response or something... %d' % end_id)
        pr = PlanResponse()
        pr.allpoints = final_list
        return final_list

    def new_scan(self, from_pose, scan):
        # add in all the new obstacles
        # rospy.loginfo('new scan')
        angle = scan.angle_min+quaternion_to_heading(from_pose.orientation)

        # rospy.loginfo('begin rotating through scan.ranges %d' % len(scan.ranges))
        # rospy.loginfo('pose x: %f y: %f' % (from_pose.position.x, from_pose.position.y,))
        count = 0
        for reading in scan.ranges:
            count += 1
            if count % 10 == 0 or count > 90:
                # rospy.loginfo('scan range %d %f' % (count, reading,))
                pass
            # add a new obstacle

            if reading > scan.range_max - .01:
                # rospy.loginfo('scan max!')
                continue
            if reading < scan.range_min + .01:
                # rospy.loginfo('scan min!')
                continue
            x = from_pose.position.x + reading*cos(angle)
            y = from_pose.position.x + reading*sin(angle)
            radius = reading*scan.angle_increment

            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y

            # rospy.loginfo('new obstacle: x: %f y: %f' % (x, y,))

            self.obstacles.add_obstacle(deepcopy(new_pose), radius)

            # rospy.loginfo('end adding obstacle')

            # check if I need to prune
            try:
                self.prune_local(new_pose, radius)
            except KeyError as ke:
                pass

            # rospy.loginfo('start prune local')

            angle += scan.angle_increment

        # check all of the points, especially nodes that might have edges
        #   passing by new obstacles
        # rospy.loginfo('prune recursive')
        self.prune_recursive()

        # self.obstacles.check_loading()

    def prune_local(self, new_pose, radius, debug=False):
        # remove any nodes in collision with a new obstacle
        # rospy.loginfo('prune local fnn: %s' % new_pose)
        nearest_id = self.find_nearest_node(new_pose)

        if debug:
            rospy.loginfo('found nearest id %d' % (nearest_id,))

        while self.distance_function(self[nearest_id], new_pose) < radius:
            self.remove_node_by_id(nearest_id)
            nearest_id = self.find_nearest_node(new_pose)

    def prune_recursive(self, root_index=0):
        # every node is on the rrt tree, so if we remove the node, we don't need
        #   to worry about any branching, it will already remove
        if self.obstacles.check_collision(self[root_index]):
            self.remove_node_by_id(root_index)
        else:
            for child_id in self[root_index].rrt_children:
                # check the half way point to each child
                # this is enough (with the ROBOT_RADIUS buffer) to pass by
                halfway_pose = Pose()
                halfway_pose.position.x = (self[root_index].position.x + self[child_id].position.x) / 2.0
                halfway_pose.position.y = (self[root_index].position.y + self[child_id].position.y) / 2.0
                #   if there is a node at halfway
                if self.obstacles.check_collision(halfway_pose):
                    # cut it out
                    self.remove_node_by_id(child_id)
                else:
                    # else call prune_recursive
                    self.prune_recursive(child_id)

    def reached_goal(self):
        if self.goal is None:
            return False
        # rospy.loginfo('reached goal fnn: %s' % self.goal)
        nearest_id = self.find_nearest_node(self.goal)
        # True if the nearest node is less than the collision check distance
        # else False
        if self.distance_function(self[nearest_id], self.goal) < .1:
            # rospy.loginfo('found a path to a goal! %d' % (nearest_id,))
            return True
        return False

    def remove_node_by_id(self, destroy_id):
        if destroy_id <= 0:
            return
        # remove child id from rrt parent
        self.remove_child_rrt(self[destroy_id].rrt_parent, destroy_id)
        
        # remove children recursively
        for child_id in self[destroy_id].rrt_children:
            self.remove_node_by_id(child_id)
        
        # once I have no children/am a rrt leaf node
        #   remove myself from my kd parent
        self.remove_child_kd(self[destroy_id].kd_parent, destroy_id)
        
        #   add all remaining kd-children back to the kd tree
        if self[destroy_id].kd_left is not None:
            self.readd_children_kd(self[destroy_id].kd_left)
        if self[destroy_id].kd_right is not None:
            self.readd_children_kd(self[destroy_id].kd_right)
        
        # remove self from the dict of points
        del self[destroy_id]

    def readd_children_kd(self, node_id):
        # add this node and all children to the kd tree again
        if self[node_id].kd_left is not None:
            self.readd_children_kd(self[node_id].kd_left)
        if self[node_id].kd_right is not None:
            self.readd_children_kd(self[node_id].kd_right)

        self.add_node_kd_it(node_id)

    def remove_child_rrt(self, parent_id, child_id):
        # remove the given child from the given parent
        ii = 0
        while ii < len(self[parent_id].rrt_children):
            if self[parent_id].rrt_children[ii] == child_id:
                del self[parent_id].rrt_children[ii]
            else:
                ii += 1

    def remove_child_kd(self, parent_id, child_id):
        # remove the given child from the given parent
        if self[parent_id].kd_left == child_id:
            self[parent_id].kd_left = None
        if self[parent_id].kd_right == child_id:
            self[parent_id].kd_right = None

    def set_goal(self, pose):
        self.goal = pose
