import math
import rospy

from collections import deque
from copy import deepcopy
from geometry_msgs.msg import Pose# , Quaternion
from math import sin, cos
# from sensor_msgs.msg import LaserScan
from utils import quaternion_to_heading, heading_to_quaternion
from utils import addv, unit, scale
from waffle.waffle_common import Planner, ROBOT_RADIUS

class Potential(Planner):
    def __init__(self):
        super(Potential, self).__init__()

    def new_scan(self, pose, scan):
        '''
        based on a new set of scan data collected at a given pose, update the
        potential field
        Input:
            Pose
            LaserScan
        Output:
            None
        '''
        pass

    def direction(self, pose):
        '''
        Gives the direction of the potential field for a given pose relative to
        the pose

        Input:
            Pose
        Output:
            Quaternion
        '''
        return pose.orientation

    def magnitude(self, pose):
        '''
        Gives the magnitude of the potential field for a given pose

        Input:
            Pose
        Output:
            float
        '''
        return abs(pose.position.x)

    def generate_plan(self, start, goal):
        deck = deque()
        deck.append(start)
        deck.append(Pose())
        deck.append(goal)
        return deck


class NaivePotential(Potential):
    '''
    An implementation of the Potential class/algorithm that assumes the only
    obstacles that exist are those it can see.

    This will likely fail without a 360 degree scan, because it will only see
    obstacles in front of it, which would "push" it backwards.
    '''

    def __init__(self):
        super(NaivePotential, self).__init__()

        self.obstacles = []

    def new_scan(self, pose, scan):
        '''
        based on a new set of scan data collected at a given pose, update the
        potential field
        Input:
            Pose
            LaserScan
        Output:
            None
        '''
        self.obstacles = []

        min_ = scan.angle_min
        max_ = scan.angle_max
        angle_inc = scan.angle_increment

        angle = min_
        index = 0

        trailing_min = scan.range_max
        trailing_min_index = -1

        for range_ in scan.ranges:
            if range_ < scan.range_max - .01:
                self.obstacles.append((range_, angle,)) # (r, theta,)
            else:
                # range is out to max, no obstacle
                pass
            angle += angle_inc
            index += 1

        self.transform_obstacles(pose)

    def transform_obstacles(self, pose):
        # take a list of r-theta obstacles and convert them to x-y, so they can
        #   be used for any position instead of just the last odom
        heading = quaternion_to_heading(pose.orientation)
        for i in xrange(0, len(self.obstacles)):
            old_obstacle = self.obstacles[i]
            x = pose.position.x + old_obstacle[0]*cos(old_obstacle[1]+heading)
            y = pose.position.y + old_obstacle[0]*sin(old_obstacle[1]+heading)
            self.obstacles[i] = (x,y,)

    def calc_potential(self, pose, debug=None):
        accum = (0,0,0,)
        for obstacle in self.obstacles:
            accum = addv(accum, self.force_vector(pose, obstacle))
        return accum

    def goal_force(self, pose, goal):
        dx = goal.position.x - pose.position.x
        dy = goal.position.y - pose.position.y

        weight = 10.0

        farce = (dx, dy, 0)
        farce = unit(farce)
        farce = scale(farce, weight)
        return farce

    def force_vector(self, pose, obstacle):

        scale_parameter = 1.0

        p_x = pose.position.x
        p_y = pose.position.y

        o_x = obstacle[0]
        o_y = obstacle[1]

        distance = math.sqrt(math.pow(p_x-o_x, 2)+math.pow(p_y-o_y, 2))
        angle = math.atan2(p_y-o_y, p_x-o_x)

        distance_from_obstacle = distance - ROBOT_RADIUS
        if distance_from_obstacle <= 0.001:
            return (1000000*(p_y-o_y)/abs(p_y-o_y), 1000000*(p_x-o_x)/abs(p_x-o_x),)
        else:
            new_magnitude = 1.0/distance_from_obstacle
            dx = new_magnitude*cos(angle)
            dy = new_magnitude*sin(angle)

        return (dx, dy, 0.0,)

    def direction(self, pose):
        '''
        Gives the direction of the potential field for a given pose relative to
        the pose

        Input:
            Pose
        Output:
            Quaternion
        '''
        vector = self.calc_potential(pose)
        return math.atan2(vector[1], vector[0])

    def magnitude(self, pose):
        '''
        Gives the magnitude of the potential field for a given pose

        Input:
            Pose
        Output:
            float
        '''
        vector = self.calc_potential(pose)
        accum = 0
        for i in xrange(0, len(vector)):
            accum += vector[i]*vector[i]
        return math.sqrt(accum)

    def generate_plan(self, start, goal, debug=None):
        if debug is not None:
            debug('potential generate plane\n'+str(goal))
        deck = deque()
        deck.append(start)

        next_ = deepcopy(start)

        distance = (math.pow(next_.position.x-goal.position.x, 2) 
            + math.pow(next_.position.y-goal.position.y, 2))

        debug('dist: %f' % (distance,))

        step_size = .1

        count = max(10, int(1.0/step_size))

        while(distance > .01 and count >= 0):
            debug('start calculating obs_force')
            obs_force = self.calc_potential(next_, debug)
            debug('end calculating obs_force')
            goal_force = self.goal_force(next_, goal)
            
            total_force = addv(obs_force, goal_force)
            total_force = unit(total_force)
            debug('%s\n%s\n%s' % (obs_force, goal_force, total_force))

            dx = total_force[0]*step_size
            dy = total_force[1]*step_size

            debug('d: %f , %f' % (dx, dy,))
            new_pose = Pose()
            new_pose.position.x = next_.position.x+dx
            new_pose.position.y = next_.position.y+dy
            new_pose.orientation = heading_to_quaternion(math.atan2(dy, dx))

            deck.append(new_pose)

            next_ = deepcopy(new_pose)

            distance = (math.pow(next_.position.x-goal.position.x, 2) 
                + math.pow(next_.position.y-goal.position.y, 2))
            debug('dist: %f' % (distance,))
            count += -1
        if (distance > .01):
            debug('count break')
        else:
            deck.append(goal)

        return list(deck)


class SavingPotential(Potential):
    '''
    An implementation of the Potential class/algorithm that keeps track of all
    of the points that it has seen a laser return from.

    This will likely fail with dynamic obstacles, because it will still see old
    obstacles even if a laser scan has shown the obstacle to have moved. With
    static obstacles, this should outperform the NaivePotential.
    '''
    #TODO(buckbaskin): implement this
    def __init__(self):
        super(SavingPotential, self).__init__()


class ImprovedPotential(Potential):
    '''
    An attempt to improve the performance of the potential field algorithm,
    (likely Saving) by trying to avoid local minima, and make other changes to
    improve its completeness, or at least the variety of geometries that it can
    handle without getting caught in local minima.
    '''
    #TODO(buckbaskin): implement this
    def __init__(self):
        super(ImprovedPotential, self).__init__()
