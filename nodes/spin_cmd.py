#!/usr/bin/env python
import roslib; roslib.load_manifest('yertle')

import sys

import rospy
from yertle.srv import *

def cmd_client():
    rospy.wait_for_service('spin_cmd')
    try:
        spin_cmd = rospy.ServiceProxy('spin_cmd', SpinCmd)
        resp = spin_cmd( int( sys.argv[1] ) )
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "Usage:\n %s cmd"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print usage()
        sys.exit(1)
    print "Sending request"
    print "Response: %s" % str(cmd_client())