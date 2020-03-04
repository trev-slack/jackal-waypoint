#!/usr/bin/env python
# Contains waypoints for jackal to follow
import rospy
import math
from std_msgs.msg import Int16
from std_msgs.msg import Float32

class Node():
	def __init__(self):
		self.location = [0.0,0.0] #current location initializes at 0,0
		self.location_goal = [0.0,0.0] #desired location initialized at 0,0

	def moveDirection(self)
	
		self.angle = math.atan((self.location_goal[0]-self.location[0])/(self.location_goal[1]-self.location[1]))
		self.distance = math.sqrt((self.location_goal[0]-self.location[0])^2+(self.location_goal[1]-self.location[1])^2)

	def waypoint(self):
		self.locationTick = rospy.Publisher('/waypoint_goal', Float32, queue_size=1)
		rospy.Subscriber('location_goal', Float32, self.moveDirection)
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			self.locationTick.publish(self.location_goal) #publish the goal location
			rospy.loginfo("Waypoint: (" + str(self.location_goal[0]) + "," + str(self.location_goal[1]))
			rate.sleep()
		rospy.spin()


if __name__ == "__main__":
	try:
		rospy.init_node('waypoint',anonymous=True)
		node = Node()
		node.waypoint()
	except rospy.ROSInterruptException:
		pass