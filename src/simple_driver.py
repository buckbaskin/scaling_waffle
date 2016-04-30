#!/usr/bin/env python

'''
Create a ROS node that uses potential fields to do automated planning
'''

import rospy

import math

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from scaling_waffle.srv import PotentialField, PotentialFieldResponse
from scaling_waffle.srv import Plan, PlanResponse
from sensor_msgs.msg import LaserScan

DRIVER = None

positions = [None]
goals = []

start = None
end = Pose()
end.position.x = 30
end.position.y = 30

def distance(pose1, pose2):
    rospy.loginfo(''+str(type(pose1))+' '+str(type(pose2)))
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y
    return math.sqrt(math.pow(x1-x2,2) + math.pow(y1-y2,2))

def odom_cb(odom):
    global start
    if start is None:
        start = odom.pose.pose
        return

    positions[0] = odom
    if len(goals) <= 0:
        # set speed to 0 and return
        if DRIVER is not None:
            rospy.loginfo('Driver: empty goals list')
            DRIVER.publish(Twist())
        return
    elif distance(odom.pose.pose, goals[0]) < .02:
        # if I'm on the goal, remove the current goal, set the speed to 0
        to_print = goals.pop(0)
        rospy.loginfo('@ '+str(to_print))
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

    ODOM_SUB = rospy.Subscriber('/odom', Odometry, odom_cb)

    rospy.loginfo('waiting for potential plan service')
    rospy.wait_for_service('/potential/plan')
    get_plan = rospy.ServiceProxy('/potential/plan', Plan)
    rospy.loginfo('found potential plan service')

    rate_limit = rospy.Rate(2)
    while start is None:
        rate_limit.sleep()

    rospy.loginfo('\n'+str(start)+'\n'+str(end))

    resp1 = get_plan(start, end)
    goals = resp1.allpoints
    rospy.loginfo('I got a %d step plan' % (len(goals)))
    waiting_for_plan = False
    
    DRIVER = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.loginfo('Driver: start simple_driver')
    rospy.spin()
    rospy.loginfo('Driver: shutdown')