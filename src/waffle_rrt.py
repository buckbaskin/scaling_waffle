#!/usr/bin/env python

'''
Create a ROS node that uses Rapid exploring Random Trees to do automated
planning
'''

import rospy

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan
from waffle.rrt import RRT

# from waffle import rrt # this is not valid by design right now

CLASSIC_WAFFLE = RRT(-5.0, 35.0, -5.0, 35.0)
CLASSIC_WAFFLE.last_pose = Pose()

def goal_cb(msg):
    # rospy.loginfo('goal cb')
    CLASSIC_WAFFLE.set_goal(msg.pose.pose)

def odom_cb(msg):
    CLASSIC_WAFFLE.last_pose = msg.pose.pose

def laser_cb(msg):
    # rospy.loginfo('laser callback')
    # rospy.loginfo('fg: %r %r' % (CLASSIC_WAFFLE.goal is None, CLASSIC_WAFFLE.reached_goal()))
    if hasattr(CLASSIC_WAFFLE, 'last_pose'):
        CLASSIC_WAFFLE.new_scan(CLASSIC_WAFFLE.last_pose, msg)
    # rospy.loginfo('end laser callback')

def reset_root(srv):
    global CLASSIC_WAFFLE
    new_root = srv.start
    CLASSIC_WAFFLE = RRT(CLASSIC_WAFFLE.minx, CLASSIC_WAFFLE.maxx, 
                            CLASSIC_WAFFLE.miny, CLASSIC_WAFFLE.maxy,
                            pose=new_root, obstacles=CLASSIC_WAFFLE.obstacles)
    CLASSIC_WAFFLE.set_goal(srv.goal)
    while not rospy.is_shutdown() and not CLASSIC_WAFFLE.reached_goal():
        CLASSIC_WAFFLE.expand_tree()
    return []

def plan_srv(dummy_srv):
    return PlanResponse(CLASSIC_WAFFLE.generate_plan())

if __name__ == '__main__':
    rospy.init_node('waffle_rrt')
    RRT_SRV = rospy.Service('/rrt/plan', Plan, plan_srv)
    RRT_RESET = rospy.Service('/rrt/reset', Plan, reset_root)
    RRT_VIS = rospy.Publisher('/rrt/visual', Odometry, queue_size=1)
    CLASSIC_WAFFLE.RRT_VIS = RRT_VIS
    GOAL_SUB = rospy.Subscriber('/rrt/goal', Odometry, goal_cb)
    ODOM_SUB = rospy.Subscriber('/odom', Odometry, odom_cb)
    LASER_SUB = rospy.Subscriber('/base_scan', LaserScan, laser_cb)

    rospy.loginfo('waffle_rrt start')
    while not rospy.is_shutdown():
        if not CLASSIC_WAFFLE.reached_goal():
            CLASSIC_WAFFLE.expand_tree()
