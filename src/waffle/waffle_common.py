from collections import deque
from geometry_msgs.msg import Pose

ROBOT_RADIUS = .5

class Planner(object):
    '''
    A generic planner class to use as a subclass to the potential and RRT
    algorithms
    '''
    def __init__(self):
        self.goal = Pose()

    def generate_plan(self, start, goal):
        '''
        Input:
            Pose start
            Pose goal
        Output:
            a deque of poses to drive to. The plan might not drive all the way
            to a goal pose in the exploratory/incomplete map case. The algorithm
            should replan if the deque runs out of items
        '''
        deck = deque()
        deck.append(start)
        deck.append(goal)
        return deck

    def set_goal(self, pose):
        '''
        Set the goal/global minimum for the potential field
        Input:
            Pose
        Output:
            None
        '''
        self.goal = pose
