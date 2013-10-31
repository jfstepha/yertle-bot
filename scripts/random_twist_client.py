#!/usr/bin/env python
import roslib; roslib.load_manifest('yertle')

import sys

import rospy
from yertle.srv import *

def random_twist_client(max_time):
    rospy.wait_for_service('random_twist')
    try:
        random_twist = rospy.ServiceProxy('random_twist', RandomTwist)
        resp1 = random_twist( max_time )
        return resp1.return_code
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        
def usage():
    return "%s [ max_time ]" % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        max_time = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting random spin %d" % max_time
    print "Return code: %s" % random_twist_client( max_time )