#!/usr/bin/env python
# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from PySide import QtGui, QtCore

##########################################################################
##########################################################################
class JoyTeleop():
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self):
    #####################################################################    
        rospy.init_node("joy_teleop");
        self.timer_rate = rospy.get_param('~publish_rate', 150)
        self.pub_twist = rospy.Publisher('twist', Twist)
        
        self.x_min = rospy.get_param("~x_min", -0.20)
        self.x_max = rospy.get_param("~x_max", 0.20)
        self.r_min = rospy.get_param("~r_min", -1.0)
        self.r_max = rospy.get_param("~r_max", 1.0)
        
        self.axis_x = rospy.get_param("~axis_x", 0)
        self.axis_y = rospy.get_param("~axis_y", 1)
        
        self.x = 0
        self.y = 0
        
        rospy.Subscriber("joy", Joy, self.joyCallback)
        
    #####################################################################    
    def spin(self):
    #####################################################################    
        r = rospy.Rate(self.timer_rate)
        while not rospy.is_shutdown():
            self.pubTwist()
            r.sleep()
    
    #####################################################################    
    def joyCallback(self, msg):
    #####################################################################    
       rospy.loginfo("-D- joystick message recieved")
       self.x = (msg.axes[self.axis_x] + 1.0) / 2.0
       self.y = (msg.axes[self.axis_y] + 1.0) / 2.0
        
    #####################################################################    
    def timerEvent(self):
    #####################################################################    
        rospy.loginfo("-D- timer tick")
        self.pubTwist()
        
    #######################################################
    def pubTwist(self):
    #######################################################
        # rospy.loginfo("publishing twist from (%0.3f,%0.3f)" %(self.x,self.y))
        self.twist = Twist()
        self.twist.linear.x = 0 - ( (1-self.y) * (self.x_max - self.x_min) + self.x_min )
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0 - ( (1-self.x) * (self.r_max - self.r_min) + self.r_min )
        
        if self.twist.linear.x > self.x_max:
            self.twist.linear.x = self.x_max
        if self.twist.linear.x < self.x_min:
            self.twist.linear.x = self.x_min
        if self.twist.angular.z > self.r_max:
            self.twist.angular.z = self.r_max
        if self.twist.angular.z < self.r_min:
            self.twist.angular.z = self.r_min
        
        self.pub_twist.publish( self.twist )
        
        
################################################
################################################
################################################
if __name__ == '__main__':
    """ main """
    joyTeleop = JoyTeleop()
    joyTeleop.spin()
        
