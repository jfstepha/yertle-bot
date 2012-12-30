#!/usr/bin/env python

"""

    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
import sys

import roslib; roslib.load_manifest('differential_drive')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String

from PySide import QtGui, QtCore
from yertle_dash_ui import *
# yertle_dash.ui was created with qt designer ("designer")
# and converted to python with
# pyside-uic yertle_dash.ui > yertle_dash_ui.py
##########################################################################
##########################################################################
class Communicate(QtCore.QObject):
##########################################################################
##########################################################################
    ltarget = QtCore.Signal(int)
    rtarget = QtCore.Signal(int)
    lvel = QtCore.Signal(int)
    rvel = QtCore.Signal(int)
    lmotor = QtCore.Signal(int)
    rmotor = QtCore.Signal(int)
    bat14v = QtCore.Signal(int)
    bat12v = QtCore.Signal(int)
    debugmsg = QtCore.Signal(str)
    lwheel = QtCore.Signal(str)
    rwheel = QtCore.Signal(str)

##########################################################################
##########################################################################
class MainWindow(QtGui.QMainWindow):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self, parent=None):
    #####################################################################    
        super(MainWindow, self).__init__()
        
        self.timer_rate = rospy.get_param('~publish_rate', 50)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        rospy.Subscriber("lwheel_vtarget", Float32, self.lwheelVtargetCallback)
        rospy.Subscriber("lwheel_vel", Float32, self.lwheelVelCallback)
        rospy.Subscriber("lmotor_cmd", Float32, self.lmotorCallback)
        
        rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        rospy.Subscriber("rwheel_vtarget", Float32, self.rwheelVtargetCallback)
        rospy.Subscriber("rwheel_vel", Float32, self.lwheelVelCallback)
        rospy.Subscriber("rmotor_cmd", Float32, self.rmotorCallback)

        rospy.Subscriber("battery", Int16, self.batCallback)
        
        rospy.Subscriber("arduino_debug", String, self.arduinoDebugCallback)
        
        self.c = Communicate()
        self.c.ltarget.connect( self.ui.pbLTarget.setValue )
        self.c.rtarget.connect( self.ui.pbRTarget.setValue )
        self.c.lvel.connect( self.ui.pbLVel.setValue )
        self.c.rvel.connect( self.ui.pbRVel.setValue )
        self.c.lmotor.connect( self.ui.pbLMotor.setValue )
        self.c.rmotor.connect( self.ui.pbRMotor.setValue )
        self.c.bat12v.connect( self.ui.pb12V.setValue )
        self.c.bat14v.connect( self.ui.pb14V.setValue )
        self.c.lwheel.connect( self.ui.tbLWheel.setText )
        self.c.rwheel.connect( self.ui.tbRWheel.setText )
        self.c.debugmsg.connect( self.ui.tbDebug.setText )
        
        
    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        self.c.lwheel.emit( str( msg.data ) ) 
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        self.c.rwheel.emit( str( msg.data ) )
        
    #############################################################################
    def lwheelVtargetCallback(self, msg):
    #############################################################################
        self.c.ltarget.emit( int( msg.data * 1000 ) ) 
        
    #############################################################################
    def rwheelVtargetCallback(self, msg):
    #############################################################################
        self.c.rtarget.emit( int( msg.data * 1000 ) )    
        
    #############################################################################
    def rwheelVelCallback(self, msg):
    #############################################################################
        self.c.rvel.emit( int( msg.data * 1000 ) )    
        
    #############################################################################
    def lwheelVelCallback(self, msg):
    #############################################################################
        self.c.lvel.emit( int( msg.data * 1000 ) )    
        
    #############################################################################
    def lmotorCallback(self, msg):
    #############################################################################
        self.c.lmotor.emit( int( msg.data  ) )    
        
    #############################################################################
    def rmotorCallback(self, msg):
    #############################################################################
        self.c.rmotor.emit( int( msg.data  ) )    
        
    #############################################################################
    def batCallback(self, msg):
    #############################################################################
        self.c.bat12v.emit( int( msg.data  ) )    
        
    #############################################################################
    def arduinoDebugCallback(self, msg):
    #############################################################################
        self.c.debugmsg.emit( msg.data )
        
##########################################################################
##########################################################################
def main():
##########################################################################
##########################################################################
    rospy.init_node('yertle_dash')
    rospy.loginfo('yertle_dash started')
    
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
