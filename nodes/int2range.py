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
   Filters IR range finder data, and scales it to meters.
   
"""
   
import rospy
import roslib
roslib.load_manifest('yertle')

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from numpy import array

############################################################################## 
############################################################################## 
class RangeFilter():
############################################################################## 
############################################################################## 

    #########################################################################
    def __init__(self):
    #########################################################################
        cur_val = 0

        self.rolling_ave = 0.0
        self.rolling_std = 0.0
        self.rolling_meters = 0.0
        rospy.init_node("range_filter")
        rospy.loginfo("-I- range_filter started")
        self.readParameters()
        
        # check for valid parameters
        # zero cannot be raised to a negative power
        if self.min_valid == 0 and self.m < 0:
            rospy.logerr("-E- range_filter: cannot have a min_valid of 0 with a negative exponent.  Forcing min_valid to 1e-10")
            min_valid = 1e-10
        
        # negative number cannot be raised to a fractional power.
        if self.min_valid < 0 and round(self.m) != self.m:
            rospy.logerr("-E- range_filter: cannot have a negative min_valid with a fractional exponent. Forcing min_valid to 0")
            self.min_valid = 0
            
        if self.max_valid < self.min_valid:
            rospy.logfatal("-E- range_filter: max_valid (%0.3f) cannot be less than min_vaid (%0.3f)" % (self.max_valid, self.min_valid)) 
            return(-1)
            
        self.prev = [0] * self.rolling_pts
    
        rospy.Subscriber("range_int", Int16, self.inputCallback)
    
        self.filtered_pub = rospy.Publisher("range_filtered", Float32)
        self.std_pub = rospy.Publisher("range_std", Float32)
        self.range_pub = rospy.Publisher("range", Range)
        
    #########################################################################
    def readParameters(self):
    #########################################################################
        self.m = rospy.get_param('~exponent', -0.9346)
        self.b = rospy.get_param('~coefficient',  0.000335597)
        self.max_valid = rospy.get_param("~max_valid", 900)
        self.min_valid = rospy.get_param("~min_valid", 1)
        self.rolling_pts = rospy.get_param('~rolling_pts',20)
        self.frame = rospy.get_param("~frame", "range0")
        
    #########################################################################
    def spin(self):
    #########################################################################
        while not rospy.is_shutdown():
            rospy.spin()
    

    #########################################################################
    def inputCallback(self, msg):
    #########################################################################
        # rospy.loginfo("-D- range_filter inputCallback")
        cur_val = msg.data
    
        if cur_val <= self.max_valid and cur_val >= self.min_valid:
            self.prev.append(cur_val)
            del self.prev[0]
        
            p = array(self.prev)
            self.rolling_ave = p.mean()
            self.rolling_std = p.std()
        
            self.rolling_meters = ((self.b * self.rolling_ave) ** self.m) / 100
        
            self.filtered_pub.publish( self.rolling_meters )
            self.std_pub.publish( self.rolling_std )
            
            range = Range()
            range.radiation_type = 1
            range.min_range = 0.04
            range.max_range = 0.3
            range.range = self.rolling_meters
            range.header.frame_id = self.frame
            range.field_of_view = 0.1
            range.header.stamp = rospy.Time.now()
            
            self.range_pub.publish( range )
            
            
            
        
    
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    """ main"""
    rangeFilter = RangeFilter()
    rangeFilter.spin()
    