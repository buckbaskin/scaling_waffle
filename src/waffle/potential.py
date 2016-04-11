import math

from collections import deque
from geometry_msgs.msg import Pose# , Quaternion
from math import sin, cos
# from sensor_msgs.msg import LaserScan
from utils import quaternion_to_heading, heading_to_quaternion
from waffle.waffle_common import Planner

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
        #TODO(buckbaskin): implement this
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
    #TODO(buckbaskin): implement this
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

    def calc_potential(self, pose):
        #TODO(buckbaskin):
        return (1,0,0,)

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

    def generate_plan(self, start, goal):
        #TODO(buckbaskin): implement this
        deck = deque()
        deck.append(start)
        deck.append(Pose())
        deck.append(goal)
        return deck


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
