from collections import deque

class Planner(object):
	'''
	A generic planner class to use as a subclass to the potential and RRT
	algorithms
	'''
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
		return deque()