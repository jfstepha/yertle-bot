# when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import roslib
roslib.load_manifest('yertle')

import os
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from ros_arduino_msgs.msg import Analog

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PySide import QtGui, QtCore
from python_qt_binding.QtCore import QTimer

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
    lapbat = QtCore.Signal(int)
    lapwifi = QtCore.Signal(int)
    bat12v = QtCore.Signal(int)
    debugmsg = QtCore.Signal(str)
    lwheel = QtCore.Signal(str)
    rwheel = QtCore.Signal(str)

##########################################################################
##########################################################################
class MyPlugin(Plugin):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self, context):
    #####################################################################    
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

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
        self.ui = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'yertle_dash.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.ui)
        # Give QObjects reasonable names
        self.ui.setObjectName('MyPluginUi')
        # Add widget to the user interface
        context.add_widget(self.ui)
        
        self.timer_rate = rospy.get_param('~publish_rate', 50)
        self.c = Communicate()
        
        self.c.ltarget.connect( self.ui.pbLTarget.setValue )
        self.c.rtarget.connect( self.ui.pbRTarget.setValue )
        self.c.lvel.connect( self.ui.pbLVel.setValue )
        self.c.rvel.connect( self.ui.pbRVel.setValue )
        self.c.lmotor.connect( self.ui.pbLMotor.setValue )
        self.c.rmotor.connect( self.ui.pbRMotor.setValue )
        self.c.bat12v.connect( self.ui.pb12V.setValue )
        self.c.lapbat.connect( self.ui.pbLapBat.setValue )
        self.c.lapwifi.connect( self.ui.pbWifi.setValue )
        self.c.lwheel.connect( self.ui.tbLWheel.setText )
        self.c.rwheel.connect( self.ui.tbRWheel.setText )
        self.c.debugmsg.connect( self.ui.tbDebug.setText )
 
        self.timer = QTimer()
        self.timer.singleShot = False
        self.timer.timeout.connect(self.timerEvent)
        self.timer.start(20) # in ms
        
        self.ticks_since_debug = 1000
        self.ticks_since_ltarg = 1000
        self.ticks_since_rtarg = 1000
        self.ticks_since_lvel = 1000
        self.ticks_since_rvel = 1000
        
        rospy.Subscriber("lwheel", Int32, self.lwheelCallback)
        rospy.Subscriber("lwheel_vtarget", Float32, self.lwheelVtargetCallback)
        rospy.Subscriber("lwheel_vel", Float32, self.lwheelVelCallback)
        rospy.Subscriber("lmotor_cmd", Float32, self.lmotorCallback)
        
        rospy.Subscriber("rwheel", Int32, self.rwheelCallback)
        rospy.Subscriber("rwheel_vtarget", Float32, self.rwheelVtargetCallback)
        rospy.Subscriber("rwheel_vel", Float32, self.rwheelVelCallback)
        rospy.Subscriber("rmotor_cmd", Float32, self.rmotorCallback)
        self.ui.pbLVel.setMinimum(0)
        self.ui.pbRVel.setMinimum(0)
        self.ui.pbLVel.setMaximum(400)
        self.ui.pbRVel.setMaximum(400)
        self.ui.pbRTarget.setMinimum(0)
        self.ui.pbLTarget.setMinimum(0)
        self.ui.pbRTarget.setMaximum(400)
        self.ui.pbLTarget.setMaximum(400)
        self.ui.pbLMotor.setMinimum(0)
        self.ui.pbRMotor.setMinimum(0)
        self.ui.pbLMotor.setMaximum(512)
        self.ui.pbRMotor.setMaximum(512)

        rospy.Subscriber("/Arduino/sensor/battery", Analog, self.batCallback)
        # 720 is pretty dead, but when motors run, it droops to <400
        self.ui.pb12V.setMaximum(770)  # 750 = ~12.8V
        self.ui.pb12V.setMinimum(600)
        
        rospy.Subscriber("laptop_bat", Int16, self.lapBatCallback)
        rospy.Subscriber("laptop_wifi", Int16, self.lapWifiCallback)
        
        
        rospy.Subscriber("Arduino/ticks", Int32, self.arduinoDebugCallback)
        
        
    #####################################################################    
    def timerEvent(self):
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
        self.c.ltarget.emit( int( msg.data * 1000 ) + 200 ) 
        
    #############################################################################
    def rwheelVtargetCallback(self, msg):
    #############################################################################
        self.ticks_since_rtarg = 0
        self.c.rtarget.emit( int( msg.data * 1000 ) + 200 )    
        
    #############################################################################
    def rwheelVelCallback(self, msg):
    #############################################################################
        self.ticks_since_rvel = 0
        self.c.rvel.emit( int( msg.data * 1000 ) + 200)    
        
    #############################################################################
    def lwheelVelCallback(self, msg):
    #############################################################################
        self.ticks_since_lvel = 0
        self.c.lvel.emit( int( msg.data * 1000 ) + 200)    
        
    #############################################################################
    def lmotorCallback(self, msg):
    #############################################################################
        self.c.lmotor.emit( int( msg.data  ) + 255 )    
        
    #############################################################################
    def rmotorCallback(self, msg):
    #############################################################################
        self.c.rmotor.emit( int( msg.data  ) + 255 )    
        
    #############################################################################
    def batCallback(self, msg):
    #############################################################################
        a= msg.value
        print "A=%s" % str(a)
        self.c.bat12v.emit( int( msg.value  ) )    
        
    #############################################################################
    def lapBatCallback(self, msg):
    #############################################################################
        self.c.lapbat.emit( int( msg.data  ) )    
        
    #############################################################################
    def lapWifiCallback(self, msg):
    #############################################################################
        self.c.lapwifi.emit( int( msg.data  ) )    
        
    #############################################################################
    def arduinoDebugCallback(self, msg):
    #############################################################################
        self.c.debugmsg.emit( "Tick %s" % str(msg.data) )
        self.ticks_since_debug = 0
        
        

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