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
 
        self.timer = QtCore.QBasicTimer()
        self.timer.rate = 10       
        self.timer.start(self.timer_rate, self)
        self.ticks_since_debug = 1000
        self.ticks_since_ltarg = 1000
        self.ticks_since_rtarg = 1000
        self.ticks_since_lvel = 1000
        self.ticks_since_rvel = 1000
        
        rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        rospy.Subscriber("lwheel_vtarget", Float32, self.lwheelVtargetCallback)
        rospy.Subscriber("lwheel_vel", Float32, self.lwheelVelCallback)
        rospy.Subscriber("lmotor_cmd", Float32, self.lmotorCallback)
        
        rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        rospy.Subscriber("rwheel_vtarget", Float32, self.rwheelVtargetCallback)
        rospy.Subscriber("rwheel_vel", Float32, self.rwheelVelCallback)
        rospy.Subscriber("rmotor_cmd", Float32, self.rmotorCallback)

        rospy.Subscriber("battery", Int16, self.batCallback)
        # 720 is pretty dead, but when motors run, it droops to <400
        self.ui.pb12V.setMaximum(770)  # 750 = ~12.8V
        self.ui.pb12V.setMinimum(730)
        
        
        rospy.Subscriber("arduino_debug", String, self.arduinoDebugCallback)
        
        
    #####################################################################    
    def timerEvent(self, event):
    #####################################################################    
        self.ticks_since_debug += 1
        if self.ticks_since_debug > 150:
            self.ui.tbDebug.setStyleSheet("QLineEdit { background-color: Red; } ")
        else: 
            self.ui.tbDebug.setStyleSheet("QLineEdit { background-color: LightGreen; } ")
       
        self.ticks_since_ltarg += 1     
        if self.ticks_since_ltarg > 10:
            self.c.ltarget.emit(0)
            
        self.ticks_since_rtarg += 1
        if self.ticks_since_rtarg > 10:
            self.c.rtarget.emit(0) 
            
        self.ticks_since_lvel += 1
        if self.ticks_since_lvel > 10:
            self.c.lvel.emit(0)
            
        self.ticks_since_rvel += 1
        if self.ticks_since_rvel > 10:
            self.c.rvel.emit(0)
                                        
        
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
        self.ticks_since_ltarg = 0
        self.c.ltarget.emit( int( msg.data * 1000 ) ) 
        
    #############################################################################
    def rwheelVtargetCallback(self, msg):
    #############################################################################
        self.ticks_since_rtarg = 0
        self.c.rtarget.emit( int( msg.data * 1000 ) )    
        
    #############################################################################
    def rwheelVelCallback(self, msg):
    #############################################################################
        self.ticks_since_rvel = 0
        self.c.rvel.emit( int( msg.data * 1000 ) )    
        
    #############################################################################
    def lwheelVelCallback(self, msg):
    #############################################################################
        self.ticks_since_lvel = 0
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
        self.ticks_since_debug = 0
        
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
