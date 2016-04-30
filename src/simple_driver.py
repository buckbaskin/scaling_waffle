#!/usr/bin/env python

'''
Create a ROS node that uses potential fields to do automated planning
'''

import rospy

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from scaling_waffle.srv import PotentialField, PotentialFieldResponse
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan

DRIVER = None

positions = [None]
goals = []

def distance(odom1, odom2):
    return 0.0

def odom_cb(odom):
    rospy.loginfo('odom callback')
    positions[0] = odom
    if len(goals) <= 0:
        # set speed to 0 and return
        if DRIVER is not None:
            rospy.loginfo('Driver: empty goals list')
            DRIVER.publish(Twist())
        return
    elif distance(odom, goals[0]) < .02:
        # if I'm on the goal, remove the current goal, set the speed to 0
        goals.pop(0)
        if DRIVER is not None:
            rospy.loginfo('Driver: next goal')
            DRIVER.publish(Twist())
        return
    else:
        # find the deltas between my position and the angle to the next goal
        # drive towards the goal position
        rospy.loginfo('Driver: moving on')
        t = Twist()

        t.linear.x = 0.0

        if t.linear.x > 0.5:
            t.linear.x = 0.5
        if t.linear.x < -0.5:
            t.linear.x = -0.5

        t.angular.z = 0.0

        if t.angular.z > 0.25:
            t.angular.z = 0.25
        if t.angular.z < -0.25:
            t.angular.z = -0.25

        DRIVER.publish(t)
        



if __name__ == '__main__':
    rospy.init_node('simple_driver')

    rospy.loginfo('waiting for potential plan service')
    rospy.wait_for_service('/potential/plan')
    get_plan = rospy.ServiceProxy('/potential/plan', Plan)
    rospy.loginfo('found potential plan service')

    resp1 = get_plan()
    goals = resp1.allpoints
    rospy.loginfo('I got a %d step plan' % (len(goals)))
    
    ODOM_SUB = rospy.Subscriber('/odom', Odometry, odom_cb)
    DRIVER = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.loginfo('Driver: start simple_driver')
    rospy.spin()
    rospy.loginfo('Driver: shutdown')