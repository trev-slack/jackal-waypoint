#!/usr/bin/env python
import rospy
import math
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from jackal_waypoint.msg import Waypoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Node():
	def __init__(self):
		self.x = 0
		self.y = 0
		self.kp = 0.5
		self.yaw = None
		self.way_x = None

	def findLoc(self, data):
		self.x = data.pose.pose.position.x 
		self.y = data.pose.pose.position.y 
		#get quaternion orientation
		orientation_q = data.pose.pose.orientation
		orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
		#convert to eulerian angles
		(self.roll,self.pitch,self.yaw) = euler_from_quaternion(orientation_list)		

	def getAngle(self, data):
		#get location of waypoint (local)
		self.way_x = data.x
		self.way_y = data.y
		#get angle to waypoint 
		if self.way_x == 0:
			if self.way_y == 0 or self.way_y > 0:
				self.target_angle = 0
			else:
				self.target_angle = -pi/2
		else:
			self.target_angle = math.atan2(self.way_y,self.way_x)

	def rotate(self):
		command = Twist()
		rospy.loginfo("target={} current={}".format(self.target_angle,self.yaw))
		command.angular.z = self.kp*(self.target_angle-self.yaw)
		self.pub_vel.publish(command)
		if abs(self.target_angle) - abs(self.yaw) <= 0.05:
			return True
		else:
			return False

	def move(self):
		#figure out how to get distance in local coords
		way_x_rel = self.way_x + self.x
		way_y_rel = self.way_y + self.y
		self.target_distance = math.sqrt((way_x_rel-self.x)**2+(way_y_rel-self.y)**2)
		command = Twist()
		rospy.loginfo("distance to target={}".format(self.target_distance))
		command.linear.x = self.kp*(self.target_distance)
		self.pub_vel.publish(command)
		if self.target_distance < 0.05:
			return True
		else: 
			return False


	def main(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.findLoc)
		rospy.Subscriber('/Waypoint', Waypoint, self.getAngle)
		self.pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			if self.yaw == None:
				print("Waiting for Odometry\n")
				continue
			if self.way_x == None:
				print("Waiting for Waypoint\n")
				continue
			check = self.rotate()
			if check:
				check2 = self.move()
				if check2:
					print("At Waypoint\n")
					break
			rate.sleep()
		rospy.spin()

if __name__ == "__main__":
	try:
		rospy.init_node('Waypoint_sub_node', anonymous=True)
		node = Node()
		node.main()
	except rospy.ROSInterruptException:
		pass