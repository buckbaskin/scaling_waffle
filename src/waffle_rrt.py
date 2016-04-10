#!/usr/bin/env python

'''
Create a ROS node that uses Rapid exploring Random Trees to do automated planning
'''

import rospy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan
from waffle.rrt import RRT

# from waffle import rrt # this is not valid by design right now

classic_waffle = RRT()
last_pose = Pose()

def goal_cb(msg):
    classic_waffle.set_goal(msg)

def odom_cb(msg):
    last_pose = msg.pose.pose

def laser_cb(msg):
    classic_waffle.new_scan(last_pose, msg)

def plan_srv(srv):
    goal = srv.goal
    start = srv.start
    return PlanResponse(classic_waffle.generate_plan(start, goal))

'''
The RRT should be iterating continually in the background, and then when it sees a new obstacle, 
it prunes the nodes that are inside it and replans if need be
'''

if __name__ == '__main__':
    rospy.init_node('waffle_potential')
    pot_srv = rospy.Service('/potential/plan', Plan, plan_srv)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
    laser_sub = rospy.Subscriber('/laser_scan', LaserScan, laser_cb)
