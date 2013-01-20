#!/usr/bin/env python
#   Copyright 2012 Jon Stephan
#   jfstepha@gmail.com
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

""" 
  Monitors laptop battery.  Requires the acpi command.  get it with:
  "sudo apt-get install acpi"
   
"""
   
import rospy
import roslib
import commands
import re
roslib.load_manifest('yertle')


from std_msgs.msg import Int16
from numpy import array

############################################################################## 
############################################################################## 
class LaptopWifi():
############################################################################## 
############################################################################## 

    #########################################################################
    def __init__(self):
    #########################################################################
        rospy.init_node("laptop_wifi")
        rospy.loginfo("-I- laptop_wifi started")
        
        self.rate = rospy.get_param('~rate',1.0)  # the rate at which to publish the transform
        self.range = rospy.get_param('~range', 5) # the maximum range of the link quality parameter

        self.bat_pub = rospy.Publisher("laptop_wifi", Int16)
        
    #########################################################################
    def read_info(self):
    #########################################################################
        # requires the acpi program.  Get it with "sudo apt-get install acpi"
        s = commands.getoutput( "iwconfig | grep Quality" )
        raw = int(re.split("Quality[=:](\d+)", s)[1])
        quality = int( 100 * raw / self.range )
        
        rospy.logdebug("-D- raw: %d quality: %d" % (raw,quality))
        
        self.bat_pub.publish( quality )
            
        
    #########################################################################
    def spin(self):
    #########################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            self.read_info() 
            r.sleep()
            
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    """ main"""
    laptopWifi = LaptopWifi()
    laptopWifi.spin()
            
