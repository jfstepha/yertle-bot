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
roslib.load_manifest('yertle')


from std_msgs.msg import Int16
from numpy import array

############################################################################## 
############################################################################## 
class LaptopBat():
############################################################################## 
############################################################################## 

    #########################################################################
    def __init__(self):
    #########################################################################
        rospy.init_node("laptop_bat")
        rospy.loginfo("-I- laptop_bat started")
        
        self.rate = rospy.get_param('~rate',1.0)  # the rate at which to publish the transform

        self.bat_pub = rospy.Publisher("laptop_bat", Int16)
        
    #########################################################################
    def read_info(self):
    #########################################################################
        # requires the acpi program.  Get it with "sudo apt-get install acpi"
        s = commands.getoutput( "acpi" )
        p = s.split(",")[1]
        bat = int(p[1:-1])
        
        rospy.logdebug("-D- percent remaining: %d" % (bat))
        
        self.bat_pub.publish( bat )
            
        
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
    laptopBat = LaptopBat()
    laptopBat.spin()
            
