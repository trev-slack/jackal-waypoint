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
		self.waypoint_x = 0
		self.waypoint_y = 0
		self.x = 0
		self.y = 0
		self.target_angle = None
		self.target_distance = None
		self.roll = 0
		self.pitch = 0
		self.yaw = None
		#depreciation variable
		self.kp = 0.5

	def findLoc(self, msg):
		#get x,y location
		self.x = msg.pose.pose.position.x 
		self.y = msg.pose.pose.position.y 
		#get quaternion orientation
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
		#convert to eulerian angles
		(self.roll,self.pitch,self.yaw) = euler_from_quaternion(orientation_list)

	def getLoc(self, way):
		#get waypoint x and y
		self.waypoint_x = way.x
		self.waypoint_y = way.y
		#convert to angle
		if self.waypoint_x == 0:
			self.target_angle = 0
		else:
			self.target_angle = math.atan((self.waypoint_y-self.y)/(self.waypoint_x-self.x)) 

	def moveLoc(self):
		command2 = Twist()
		rate = rospy.Rate(120) 
		self.target_distance = math.sqrt((self.x-self.waypoint_x)**2+(self.y-self.waypoint_y)**2)
		print(self.target_distance)
		while not self.target_distance <= 0.01:
			self.target_distance = math.sqrt((self.x-self.waypoint_x)**2+(self.y-self.waypoint_y)**2)
			command2.linear.x = self.kp*(self.target_distance)
			if self.waypoint_x < 0 and self.waypoint_y < 0:
				command2.linear.x = -self.kp*(self.target_distance)
			self.pub_vel.publish(command2)
			rospy.loginfo("target={},{} current={},{} distance={}".format(self.waypoint_x,self.waypoint_y,self.x,self.y,self.target_distance))
		 	if abs(self.x) > abs(self.waypoint_x) and abs(self.y) > abs(self.waypoint_y) or self.target_distance<0.01:
		 		print("waypoint Reached!\n")
		 		break
		 	rate.sleep()
		self.main()

	def main(self):
		rospy.Subscriber("/odometry/filtered", Odometry, self.findLoc)
		rospy.Subscriber('/Waypoint', Waypoint, self.getLoc)
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		#go to right angle
		rate = rospy.Rate(120) 
		command = Twist()
		check = False
		while not rospy.is_shutdown():
			#check if there is a waypoint
			if self.target_angle == None or self.yaw == None:
				print("Waiting for waypoint.\n")
				continue
			#check if at waypoint
			if abs(self.x)>=abs(self.waypoint_x)-0.1 and abs(self.y)<=abs(self.waypoint_x)+0.1 and abs(self.y)>=abs(self.waypoint_y)-0.1 and abs(self.y)<=abs(self.waypoint_y)+0.1:
				if check == False:
					print("At waypoint\n")
					check = True
				continue				
			# if self.target_distance != None and self.target_distance<0.01 or :
			# 	if check == False:
			# 		print("At waypoint\n")
			# 		check = True
			# 	continue
			#get angular velocity
			command.angular.z = self.kp*(self.target_angle-self.yaw)
			self.pub_vel.publish(command)
			rospy.loginfo("target={} current={}".format(self.target_angle,self.yaw))
			#break condition if within 1% of angle
			if self.target_angle == 0 and abs(self.yaw)<= 0.05:
				print("broke 0 \n")
				self.moveLoc()
			elif abs(abs(self.target_angle)-abs(self.yaw)) <= 0.05*abs(self.target_angle):
				print("broke\n")
				self.moveLoc()
			rate.sleep()
		rospy.spin()

if __name__ == "__main__":
	try:
		rospy.init_node('Waypoint_sub_node', anonymous=True)
		node = Node()
		node.main()
	except rospy.ROSInterruptException:
		pass
