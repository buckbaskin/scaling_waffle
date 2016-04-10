from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan
from waffle_common import Planner

class Potential(Planner):
    def __init__(self):
        self.goal = Pose()
        

    def set_goal(self, pose):
        '''
        Set the goal/global minimum for the potential field
        Input:
            Pose
        Output:
            None
        '''
        self.goal = pose

    def new_scan(self, pose, scan):
        '''
        based on a new set of scan data collected at a given pose, update the potential field
        Input:
            Pose
            LaserScan
        Output:
            None
        '''
        pass

    def direction(self, pose):
        '''
        Gives the direction of the potential field for a given pose relative to the pose

        Input:
            Pose
        Output:
            Quaternion
        '''
        return Quaternion()

    def magnitude(self, pose):
        '''
        Gives the magnitude of the potential field for a given pose

        Input:
            Pose
        Output:
            float
        '''
        return 0.0


class NaivePotential(Potential):
    '''
    An implementation of the Potential class/algorithm that assumes the only
    obstacles that exist are those it can see.

    This will likely fail without a 360 degree scan, because it will only see
    obstacles in front of it, which would "push" it backwards.
    '''
    pass


class SavingPotential(Potential):
    '''
    An implementation of the Potential class/algorithm that keeps track of all
    of the points that it has seen a laser return from.

    This will likely fail with dynamic obstacles, because it will still see old
    obstacles even if a laser scan has shown the obstacle to have moved. With
    static obstacles, this should outperform the NaivePotential.
    '''
    pass


class ImprovedPotential(Potential):
    '''
    An attempt to improve the performance of the potential field algorithm,
    (likely Saving) by trying to avoid local minima, and make other changes to
    improve its completeness, or at least the variety of geometries that it can 
    handle without getting caught in local minima.
    '''
    pass

del Potential