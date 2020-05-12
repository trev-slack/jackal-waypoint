#!/usr/bin/env python
import rospy
import rviz
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QSlider, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout
from PyQt5.QtCore import QSize, Qt, pyqtSlot, pyqtSignal

#GUI class
class myWidget( QWidget ):
    pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
    def __init__(self):
        QWidget.__init__(self)
        #intial linear and angular speed
        self.lin = 1
        self.ang = 1
        #ROS publisher
        #main RViz window
        self.frame = rviz.VisualizationFrame()
        #remove loading splash
        self.frame.setSplashPath( "" )
        #initialize manager
        self.frame.initialize()
        #read/run config
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "config.myviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        #diable menus
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        #set up manager for RViz
        self.manager = self.frame.getManager()
        #reference to display
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        
        #layout
        rviz_layout = QVBoxLayout()
        rviz_layout.addWidget( self.frame )
        
        view_layout = QHBoxLayout()
        
        toppushButton = QtWidgets.QPushButton( "Top View" )
        toppushButton.clicked.connect( self.onTopButtonClick )
        view_layout.addWidget( toppushButton )
        
        sidepushButton = QtWidgets.QPushButton( "Side View" )
        sidepushButton.clicked.connect( self.onSideButtonClick )
        view_layout.addWidget( sidepushButton )
        
        rviz_layout.addLayout( view_layout )
        
        control_box = QGridLayout()
        control_box.setColumnStretch(1, 4)
        control_box.setColumnStretch(2, 4)
        

        pushButtonForwards = QtWidgets.QPushButton(self)
        pushButtonForwards.setObjectName("pushButtonForwards")
        pushButtonForwards.setText("Forwards")
        pushButtonForwards.setAutoRepeat(True)

        pushButtonBackwards = QtWidgets.QPushButton(self)
        pushButtonBackwards.setObjectName("pushButtonBackwards")
        pushButtonBackwards.setText("Backwards")
        pushButtonBackwards.setAutoRepeat(True)

        pushButtonLeft = QtWidgets.QPushButton(self)
        pushButtonLeft.setObjectName("pushButtonLeft")
        pushButtonLeft.setText("Left")
        pushButtonLeft.setAutoRepeat(True)

        pushButtonRight = QtWidgets.QPushButton(self)
        pushButtonRight.setObjectName("pushButtonRight")
        pushButtonRight.setText("Right")
        pushButtonRight.setAutoRepeat(True)

        #control_box.addWidget(QtWidgets.QPushButton('1'),0,0)
        control_box.addWidget(pushButtonForwards,0,1)
        #control_box.addWidget(QtWidgets.QPushButton('3'),0,2)
        control_box.addWidget(pushButtonLeft,1,0)
        #control_box.addWidget(QtWidgets.QPushButton('5'),1,1)
        control_box.addWidget(pushButtonRight,1,2)
        #control_box.addWidget(QtWidgets.QPushButton('7'),2,0)
        control_box.addWidget(pushButtonBackwards,2,1)
        #control_box.addWidget(QtWidgets.QPushButton('9'),2,2)        

        speed_box = QGridLayout()

        self.qdialspeed = QtWidgets.QDial(self)
        self.qdialspeed.setObjectName("QDialSpeed")
        self.qdialspeed.setMinimum(1)
        self.qdialspeed.setMaximum(10)
        self.qdialspeed.setValue(1)
        self.qdialspeed.setNotchesVisible(True)
        speed_box.addWidget(self.qdialspeed,0,0)

        self.qdialAngspeed = QtWidgets.QDial(self)
        self.qdialAngspeed.setObjectName("QDialAngSpeed")
        self.qdialAngspeed.setMinimum(1)
        self.qdialAngspeed.setMaximum(10)
        self.qdialAngspeed.setValue(1)
        self.qdialAngspeed.setNotchesVisible(True)
        speed_box.addWidget(self.qdialAngspeed,0,1)

        self.labelVel = QtWidgets.QLabel(self)
        self.labelVel.setObjectName("labelVel")
        self.labelVel.setText("Linear Speed : 1")
        self.labelVel.adjustSize()
        speed_box.addWidget(self.labelVel,1,0)

        self.labelAngVel = QtWidgets.QLabel(self)
        self.labelAngVel.setObjectName("labelAngVel")
        self.labelAngVel.setText("Angular Speed : 1")
        self.labelAngVel.adjustSize()
        speed_box.addWidget(self.labelAngVel,1,1)



        main_layout = QGridLayout()
        main_layout.addLayout(rviz_layout,0,0,0,1)
        main_layout.addLayout(control_box,0,1)
        main_layout.addLayout(speed_box,1,1)




        self.setLayout( main_layout )


        #behavior
        pushButtonForwards.pressed.connect(self.moveForward)
        pushButtonBackwards.pressed.connect(self.moveBackwards)
        pushButtonLeft.pressed.connect(self.moveLeft)
        pushButtonRight.pressed.connect(self.moveRight)
        self.qdialspeed.sliderReleased.connect(self.updateLin)
        self.qdialAngspeed.sliderReleased.connect(self.updateAng)

    def updateLabel(self):
        self.labelVel.setText("Linear Speed : " + str(self.qdialspeed.value()))
        self.labelVel.adjustSize()
    def updateLabelAng(self):
        self.labelAngVel.setText("Angular Speed : " + str(self.qdialAngspeed.value()))
        self.labelAngVel.adjustSize()


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
        command.angular.z = self.ang*1
        self.pub_vel.publish(command)

    def moveRight(self):
        command = Twist()
        command.angular.z = self.ang*-1
        self.pub_vel.publish(command)

    def updateLin(self):
        self.lin = self.qdialspeed.value()

    def updateAng(self):
        self.ang = self.qdialAngspeed.value()

    def onTopButtonClick( self ):
        self.switchToView( "Top View" );
        
    def onSideButtonClick( self ):
        self.switchToView( "Side View" );
        
    def switchToView( self, view_name ):
        view_man = self.manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )


if __name__ == '__main__':
    try:
        rospy.init_node('Waypoint_sub_node', anonymous=True)

        app = QtWidgets.QApplication([])

        myviz = myWidget()
        myviz.resize( 500, 500 )
        myviz.show()

        app.exec_()
    except rospy.ROSInterruptException:
        pass