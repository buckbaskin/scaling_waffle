#!/usr/bin/env python

'''
Create a ROS node that uses potential fields to do automated planning
'''

import rospy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from scaling_waffle.srv import PotentialField, PotentialFieldResponse
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan
from waffle.potential import NaivePotential
# from waffle.potential import SavingPotential
# from waffle.potential import ImprovedPotential

# from waffle import potential # this is not valid by design right now

CLASSIC_WAFFLE = NaivePotential()
CLASSIC_WAFFLE.last_pose = Pose()

def goal_cb(msg):
    CLASSIC_WAFFLE.set_goal(msg)

def odom_cb(msg):
    CLASSIC_WAFFLE.last_pose = msg.pose.pose

def laser_cb(msg):
    CLASSIC_WAFFLE.new_scan(CLASSIC_WAFFLE.last_pose, msg)

def potential_srv(srv):
    pfr = PotentialFieldResponse()
    pfr.direction = CLASSIC_WAFFLE.direction(srv.pose)
    pfr.magnitude = CLASSIC_WAFFLE.magnitude(srv.pose)
    return pfr

def plan_srv(srv):
    pr = PlanResponse()
    plan = CLASSIC_WAFFLE.generate_plan(srv.start, srv.goal)
    pr.allpoints = plan
    return pr

if __name__ == '__main__':
    rospy.init_node('waffle_potential')
    POTEN_SRV = rospy.Service('/potential/field', PotentialField, potential_srv)
    PLAN_SRV = rospy.Service('/potential/plan', Plan, plan_srv)
    ODOM_SUB = rospy.Subscriber('/odom', Odometry, odom_cb)
    LASER_SUB = rospy.Subscriber('/laser_scan', LaserScan, laser_cb)
    rospy.loginfo('start waffle_potential')
    rospy.spin()
