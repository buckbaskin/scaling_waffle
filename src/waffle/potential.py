from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import LaserScan

class Potential(object):
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