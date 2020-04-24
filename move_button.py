#!/usr/bin/env python
import rospy
import math
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QSlider, QVBoxLayout
from PyQt5.QtCore import QSize, Qt, pyqtSlot,pyqtSignal

class UI_Movedirection(QMainWindow):
	def setupUi(self):
		QMainWindow.__init__(self)
		self.setWindowTitle("Jackal")
		self.setMinimumSize(QSize(640, 480))   
		self.title = QLabel("Controller", self)
		self.title.setAlignment(QtCore.Qt.AlignCenter) 

		self.pushButtonForwards = QtWidgets.QPushButton(self)
		self.pushButtonForwards.setGeometry(QtCore.QRect(140, 100, 99, 27))#x,y, width, height
		self.pushButtonForwards.setObjectName("pushButtonForwards")
		self.pushButtonForwards.setText("Forwards")
		self.pushButtonForwards.setAutoRepeat(True)

		self.pushButtonBackwards = QtWidgets.QPushButton(self)
		self.pushButtonBackwards.setGeometry(QtCore.QRect(140, 200, 99, 27))
		self.pushButtonBackwards.setObjectName("pushButtonBackwards")
		self.pushButtonBackwards.setText("Backwards")
		self.pushButtonBackwards.setAutoRepeat(True)

		self.pushButtonLeft = QtWidgets.QPushButton(self)
		self.pushButtonLeft.setGeometry(QtCore.QRect(70, 150, 99, 27))
		self.pushButtonLeft.setObjectName("pushButtonLeft")
		self.pushButtonLeft.setText("Left")
		self.pushButtonLeft.setAutoRepeat(True)

		self.pushButtonRight = QtWidgets.QPushButton(self)
		self.pushButtonRight.setGeometry(QtCore.QRect(210, 150, 99, 27))
		self.pushButtonRight.setObjectName("pushButtonRight")
		self.pushButtonRight.setText("Right")
		self.pushButtonRight.setAutoRepeat(True)

		self.qdialspeed = QtWidgets.QDial(self)
		self.qdialspeed.setObjectName("QDialSpeed")
		self.qdialspeed.setMinimum(1)
		self.qdialspeed.setMaximum(10)
		self.qdialspeed.setValue(1)
		self.qdialspeed.setGeometry(QtCore.QRect(350, 100, 99, 99))
		self.qdialspeed.setNotchesVisible(True)

		self.label = QtWidgets.QLabel(self)
		self.label.setObjectName("label")
		self.label.move(365,200)
		self.label.setText(" Speed : 1")
		self.qdialspeed.sliderReleased.connect(self.updateLabel)

	def updateLabel(self):
		self.label.setText(' Speed : ' + str(self.qdialspeed.value()))


class Widget(UI_Movedirection):

	pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)

	def __init__(self):
		self.lin = 1

		self.setupUi()

		self.pushButtonForwards.pressed.connect(self.moveForward)
		self.pushButtonBackwards.pressed.connect(self.moveBackwards)
		self.pushButtonLeft.pressed.connect(self.moveLeft)
		self.pushButtonRight.pressed.connect(self.moveRight)

		self.qdialspeed.sliderReleased.connect(self.updateLin)

	def moveForward(self):
		command = Twist()
		command.linear.x = self.lin
		self.pub_vel.publish(command)

	def moveBackwards(self):
		command = Twist()
		command.linear.x = self.lin*-1
		self.pub_vel.publish(command)

	def moveLeft(self):
		command = Twist()
		command.angular.z = 1
		self.pub_vel.publish(command)

	def moveRight(self):
		command = Twist()
		command.angular.z = -1
		self.pub_vel.publish(command)

	def updateLin(self):
		self.lin = self.qdialspeed.value()



if __name__ == "__main__":
	try:
		rospy.init_node('Waypoint_sub_node', anonymous=True)
		app = QtWidgets.QApplication([])
		mainWin = Widget()
		mainWin.show()
		app.exec_()
	except rospy.ROSInterruptException:
		pass