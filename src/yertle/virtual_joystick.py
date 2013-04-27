# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import roslib
roslib.load_manifest('yertle')

import os
import rospy
from geometry_msgs.msg import Twist

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PySide import QtGui, QtCore
from python_qt_binding.QtCore import QTimer

##########################################################################
##########################################################################
class MainVirtJoy(QWidget):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self):
    #####################################################################    
        super(MainVirtJoy, self).__init__()
        self.timer_rate = rospy.get_param('~publish_rate', 150)
        self.pub_twist = rospy.Publisher('twist', Twist)
        
        self.x_min = rospy.get_param("~x_min", -0.10)
        self.x_max = rospy.get_param("~x_max", 0.10)
        self.r_min = rospy.get_param("~r_min", -1.0)
        self.r_max = rospy.get_param("~r_max", 1.0)
        
        self.timer = QTimer()
        self.timer.singleShot = False
        self.timer.timeout.connect(self.timerEvent)
        
   #####################################################################    
    def mousePressEvent(self, event):
    #####################################################################    
        rospy.logdebug('-D- virtual_joystick mouse clicked')
        self.timer.start(self.timer_rate)
        self.get_position(event)
        
    #####################################################################    
    def mouseReleaseEvent(self, event):
    #####################################################################    
        rospy.logdebug('-D- virtual_joystick mouse released')
        self.x = 0.5
        self.y = 0.5
        self.pubTwist()
        self.timer.stop()
        
    #####################################################################    
    def mouseMoveEvent(self, event):
    #####################################################################    
        self.get_position(event)
        
    #####################################################################    
    def get_position(self, event):
    #####################################################################    
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h
        
        rospy.logdebug('-D- virtual_joystick point (%0.2f, %0.2f)' % (self.x,self.y))
        
    #####################################################################    
    def timerEvent(self):
    #####################################################################    
        rospy.logdebug("-D- virtual_joystick timer event")
        self.pubTwist()
        
    #######################################################
    def pubTwist(self):
    #######################################################
        # rospy.loginfo("publishing twist from (%0.3f,%0.3f)" %(self.x,self.y))
        self.twist = Twist()
        self.twist.linear.x = (1-self.y) * (self.x_max - self.x_min) + self.x_min
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = (1-self.x) * (self.r_max - self.r_min) + self.r_min
        
        if self.twist.linear.x > self.x_max:
            self.twist.linear.x = self.x_max
        if self.twist.linear.x < self.x_min:
            self.twist.linear.x = self.x_min
        if self.twist.angular.z > self.r_max:
            self.twist.angular.z = self.r_max
        if self.twist.angular.z < self.r_min:
            self.twist.angular.z = self.r_min
        
        self.pub_twist.publish( self.twist )
        

##########################################################################
##########################################################################
class VirtualJoystick(Plugin):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self, context):
    #####################################################################    
        super(VirtualJoystick, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('VirtualJoystick')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self.ui = MainVirtJoy()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'virtual_joystick.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.ui)
        # Give QObjects reasonable names
        self.ui.setObjectName('MyPluginUi')
        # Add widget to the user interface
        context.add_widget(self.ui)
        
        rospy.loginfo("-I- virtual_joystick plugin started")
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog