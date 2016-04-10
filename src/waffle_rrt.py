#!/usr/bin/env python

'''
Create a ROS node that uses Rapid exploring Random Trees to do automated
planning
'''

import rospy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan
from waffle.rrt import RRTBase

# from waffle import rrt # this is not valid by design right now

CLASSIC_WAFFLE = RRTBase()
CLASSIC_WAFFLE.last_pose = Pose()

def goal_cb(msg):
    CLASSIC_WAFFLE.set_goal(msg)

def odom_cb(msg):
    CLASSIC_WAFFLE.last_pose = msg.pose.pose

def laser_cb(msg):
    CLASSIC_WAFFLE.new_scan(CLASSIC_WAFFLE.last_pose, msg)

def plan_srv(srv):
    goal = srv.goal
    start = srv.start
    return PlanResponse(CLASSIC_WAFFLE.generate_plan(start, goal))

if __name__ == '__main__':
    rospy.init_node('waffle_rrt')
    RRT_SRV = rospy.Service('/rrt/plan', Plan, plan_srv)
    ODOM_SUB = rospy.Subscriber('/odom', Odometry, odom_cb)
    LASER_SUB = rospy.Subscriber('/laser_scan', LaserScan, laser_cb)

    while not rospy.is_shutdown():
        CLASSIC_WAFFLE.expand_tree()
