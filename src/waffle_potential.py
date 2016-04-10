#!/usr/bin/env python

'''
Create a ROS node that uses potential fields to do automated planning
'''

import rospy

from nav_msgs.msg import Odometry
from scaling_waffle.srv import PotentialField, PotentialFieldResponse
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan
from waffle.potential import NaivePotential
from waffle.potential import SavingPotential
from waffle.potential import ImprovedPotential

# from waffle import potential # this is not valid by design right now

classic_waffle = NaivePotential()
last_pose = Odometry()

def goal_cb(msg):
    classic_waffle.set_goal(msg)

def odom_cb(msg):
    last_pose = msg

def laser_cb(msg):
    classic_waffle.new_scan(last_pose, msg)

def potential_srv(srv):
    return PotentialFieldResponse(classic_waffle.direction(), classic_waffle.magnitude())

def plan_srv(srv):
    goal = srv.goal
    start = srv.start
    return PlanResponse(classic_waffle.generate_plan(start, goal))

if __name__ == '__main__':
    rospy.init_node('waffle_potential')
    pot_srv = rospy.Service('/potential/field', PotentialField, potential_srv)
    pot_srv = rospy.Service('/potential/plan', Plan, plan_srv)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
    laser_sub = rospy.Subscriber('/laser_scan', LaserScan, laser_cb)