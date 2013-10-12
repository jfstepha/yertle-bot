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

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from yertle.srv import *
from tf import transformations
import math

############################################################################## 
############################################################################## 
class SpinFilter():
############################################################################## 
############################################################################## 

    #########################################################################
    def __init__(self):
    #########################################################################
        self.laser = LaserScan()
        self.main_publishing = True
        self.accumulating = False
        self.current_orientation = 0
        self.starting_orientation = 0
        self.min_orientation = 999
        self.max_orientation = -999
        self.prev_orientation = 999
        self.wrap = 0

        self.accum_init()

        rospy.init_node("range_filter")
        rospy.loginfo("-I- spin_filter started")
       
        rospy.Subscriber("scan", LaserScan, self.laserScanCallback) 
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.laser_pub = rospy.Publisher("scan_filtered", LaserScan)
        self.tmp_laser_pub = rospy.Publisher("scan_tmp", LaserScan)
        s1 = rospy.Service('spin_cmd', SpinCmd, self.handle_spin_cmd)
        
    #########################################################################
    def odomCallback(self, msg):
    #########################################################################
        #self.current_orientation = msg.pose.pose.orientation.z
        (stupid, stupid, self.current_orientation) = transformations.euler_from_quaternion([0,0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        if self.prev_orientation == 999:
            self.prev_orientation = self.current_orientation
        elif self.prev_orientation > 2.0 and self.current_orientation < -2.0:
            self.wrap = self.wrap + 1
        elif self.prev_orientation < -2.0 and self.current_orientation > 2.0:
            self.wrap = self.wrap - 1
        self.current_orientation = self.current_orientation + self.wrap * 2 * math.pi
        
        print "-D- spin_filter orientation: %0.3f" % self.current_orientation
        
        

    #########################################################################
    def laserScanCallback(self, msg): 
    #########################################################################
        self.laser = msg
        # print "-D- spin filter recieved %s" % str( msg )
        if self.main_publishing:
            self.laser_pub.publish( self.laser )
        if self.accumulating:
            self.accumulate( msg )
            
    #########################################################################
    def accum_init(self):
    #########################################################################
        self.tmp_scan = LaserScan()
        self.starting_orientation = self.current_orientation
        print "-I- spin_filter starting orientation: %d" % self.starting_orientation
            
    #########################################################################
    def accumulate(self, msg):
    ######################################################################### * 
        scan_width = msg.angle_max - msg.angle_min

        if len(self.tmp_scan.ranges) == 0:
            self.tmp_scan.ranges = msg.ranges
        if self.min_orientation == 999:
            print "-D- scan_filter initial scan"
            self.max_orientation = self.current_orientation
            self.min_orientation = self.current_orientation
            self.tmp_scan.angle_min = msg.angle_min
            self.tmp_scan.angle_max = msg.angle_max
            self.tmp_scan.ranges = msg.ranges
        else:
            if self.current_orientation > self.max_orientation:
                print "-D- scan filter max increasing from %0.3f to %0.3f" % (self.max_orientation, self.current_orientation)
                incflag=True
                delta_o = self.current_orientation - self.max_orientation
                new_pts = int( delta_o / msg.angle_increment )
                max_i = len( msg.ranges )
                if new_pts > max_i:
                    new_pts = max_i
                if new_pts > 0:
                    self.max_orientation = self.current_orientation
                print "-D- inc delta_o %0.3f angle_increment: %0.3f new_pts %d" % ( delta_o, msg.angle_increment, new_pts)
                self.tmp_scan.angle_min = self.tmp_scan.angle_min - new_pts * msg.angle_increment
                # import pdb; pdb.set_trace()  ## DEBUG ##
                for i in range( new_pts ):
                    print "-D- scan_filter increase appending %s" % str( msg.ranges[ max_i - new_pts + i])
                    self.tmp_scan.ranges = self.tmp_scan.ranges + ( msg.ranges[ max_i - new_pts + i ], )
    
            if self.current_orientation < self.min_orientation:
                print "-D- scan filter min decreasing from %0.3f to %0.3f" % (self.min_orientation, self.current_orientation)
                delta_o = self.min_orientation - self.current_orientation
                new_pts = int( abs( delta_o ) / msg.angle_increment )
                if new_pts > 0:
                    self.min_orientation = self.current_orientation
                max_i = len( msg.ranges )
                if new_pts > max_i:
                    new_pts = max_i
                print "-D- dec delta_o %0.3f angle_increment: %0.3f new_pts %d" % ( delta_o, msg.angle_increment, new_pts)
                self.tmp_scan.angle_max = self.tmp_scan.angle_max + new_pts * msg.angle_increment
                b =  ()
                for i in range( new_pts ):
                    print "-D- scan_filter decrease appending %s" % str( msg.ranges[ max_i - new_pts + i])
                    b = b + ( msg.ranges[i], )
                # b.append( self.tmp_scan.ranges )
                self.tmp_scan.ranges = b + self.tmp_scan.ranges
            
        
        #self.tmp_scan.ranges = msg.ranges
        self.tmp_scan.header = msg.header
        self.tmp_scan.time_increment = msg.time_increment
        self.tmp_scan.range_min = msg.range_min
        self.tmp_scan.range_max = msg.range_max
        #self.tmp_scan.angle_min = msg.angle_min
        #self.tmp_scan.angle_max = msg.angle_max
        self.tmp_scan.angle_increment = msg.angle_increment
        self.tmp_scan.intensities = msg.intensities
        self.tmp_scan.scan_time = msg.scan_time
        self.tmp_scan.header.stamp = rospy.Time.now()
        
        print "-D- scan_filter angle_min %0.3f angle_max %0.3f"  % (self.tmp_scan.angle_min, self.tmp_scan.angle_max)
        #print "-D- scan_filter angle_min %0.3f angle_max %0.3f ranges %s "   % (self.tmp_scan.angle_min, self.tmp_scan.angle_max, self.tmp_scan.ranges )
        
        self.tmp_laser_pub.publish( self.tmp_scan )

    #########################################################################
    def handle_spin_cmd(self, req):
    #########################################################################
        print "-I- spin_filter received command %s" % str( req )
        commands = { 0 : self.cmd_none,
                 1 : self.cmd_pause,
                 2 : self.cmd_resume,
                 3 : self.cmd_accumulate,
                 4 : self.cmd_publish,
                 5 : self.cmd_cancel}
        retval = commands[ int( req.cmd ) ]()
        return retval
    
    #########################################################################
    def cmd_none(self):
    #########################################################################
        print "-I- spin filter none command"
        return 0

    #########################################################################
    def cmd_pause(self):
    #########################################################################
        print "-I- spin_filter pausing"
        self.main_publishing = False
        return 0

    #########################################################################
    def cmd_resume(self):
    #########################################################################
        print "-I- spin_filter resuming"
        self.main_publishing = True
        return 0

    #########################################################################
    def cmd_accumulate(self):
    #########################################################################
        print "-I- spin_filter accumulating"
        self.accum_init()
        self.main_publishing = False
        self.accumulating = True
        return 0

    #########################################################################
    def cmd_publish(self):
    #########################################################################
        print "-I- spin_filter publishing"
        self.laser_pub.publish( self.tmp_scan )
        self.accumulating = False
        self.main_publishing = True
        return 0

    #########################################################################
    def cmd_cancel(self):
    #########################################################################
        print "-I- spin_filter cancel"
        self.main_publishing = True
        self.accumulating = False
        return 0
        

    #########################################################################
    def spin(self):
    #########################################################################
        while not rospy.is_shutdown():
            rospy.spin()
        
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    """ main"""
    spinFilter = SpinFilter()
    spinFilter.spin()
    
