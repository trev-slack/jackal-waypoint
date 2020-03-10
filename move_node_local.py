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
		#odometry x,y
		self.x = 0
		self.y = 0
		#rate scaling factor
		self.kp = 0.5
		#jackal yaw angle
		self.yaw = None
		#waypoint x location
		self.way_x = None
		#distance to target
		self.target_distance = 1000000
		self.old_distance = None
		#previous waypoint location
		self.old_way_x = None
		self.old_way_y = None


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
		self.target_angle = math.atan2(self.way_y,self.way_x)

	def rotate(self):
		command = Twist()
		rospy.loginfo("target angle={} current angle={}".format(self.target_angle,self.yaw))
		#rotate jackal
		command.angular.z = self.kp*(self.target_angle-self.yaw)
		self.pub_vel.publish(command)
		#check if at target_angle
		if self.target_angle < 0:
			if self.target_angle - self.yaw >= -0.01:
				return True
		elif self.target_angle > 0:
			if self.target_angle - self.yaw <= 0.01:
				return True
		elif self.target_angle == 0:
			if self.target_angle + abs(self.yaw) <= 0.01:
				return True
		else:
			return False

	def move(self):
		self.old_distance = self.target_distance
		#get local distance
		way_x_rel = self.way_x + self.x_curr
		way_y_rel = self.way_y + self.y_curr
		self.target_distance = math.sqrt((way_x_rel-self.x)**2+(way_y_rel-self.y)**2)
		command = Twist()
		rospy.loginfo("distance to target={}".format(self.target_distance))
		#move jackal, velocity scaled by distance
		command.linear.x = math.log(self.target_distance+1)
		self.pub_vel.publish(command)
		#check if at waypoint
		if self.target_distance > self.old_distance:
			self.check = self.check+1
		if self.target_distance < 0.03 or self.check >= 5:
			return True
		else: 
			return False

	def main(self):
		#subscribe and get odometry location and waypoint data
		rospy.Subscriber("/odometry/filtered", Odometry, self.findLoc)
		rospy.Subscriber('/Waypoint', Waypoint, self.getAngle)
		#create publisher for cmd_vel to move jackal
		self.pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
		rate = rospy.Rate(60)
		#text formating
		flag_format = False
		flag2 = False
		flag3 = False
		#loop to rotate and move
		while not rospy.is_shutdown():
			#check if have odometry data
			if self.yaw == None:
				if flag3 == False:
					flag3 = True
					print("Waiting for Odometry.")
				continue
			#check if have waypoint data
			if self.way_x == None:
				if flag2 == False:
					rospy.loginfo("Approximate Global (x,y)=({},{})".format(self.x,self.y))
					print("Waiting for Waypoint.")
					flag2 = True
				continue
			#check if have a new, nonzero waypoint
			if (self.old_way_x == self.way_x and self.old_way_y == self.way_y) or (self.way_x==0 and self.way_y==0):
				if flag_format == False:
					print("Waiting for new waypoint.\n")
					flag_format = True
				continue
			#call rotate function
			flag_rotate = self.rotate()
			#if done rotating
			if flag_rotate:
				flag_move = False
				self.x_curr = self.x
				self.y_curr = self.y
				#call move function
				self.check = 0
				while not flag_move and not rospy.is_shutdown():
					flag_move = self.move()
				#now at waypoint
				print("At Waypoint.\n")
				rospy.loginfo("Approximate Global (x,y)=({},{})".format(self.x,self.y))
				self.old_way_x = self.way_x
				self.old_way_y = self.way_y
				#recall main
				self.main()
			rate.sleep()
		rospy.spin()

if __name__ == "__main__":
	try:
		rospy.init_node('Waypoint_sub_node', anonymous=True)
		node = Node()
		print("=====Jackal Move to Waypoint=====")
		node.main()
	except rospy.ROSInterruptException:
		pass