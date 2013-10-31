#!/usr/bin/env python
import roslib; roslib.load_manifest('yertle')
from yertle.srv import *
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import random

pub_twist = rospy.Publisher('twist', Twist)

rng_center = 0;

def rangeCallback(msg):
    global rng_center
    rng_center = msg.range;

rospy.Subscriber("range_center", Range, rangeCallback)

def pubTwist(dir):
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = dir
        
    pub_twist.publish( twist )

def pubStraight(speed):
    twist = Twist()
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
        
    pub_twist.publish( twist )

def handle_random_twist(req):
    print "-D- handl_random_twist Server received %s" % str(req.max_time)
    retval = random_turn( req.max_time )
    pubStraight(0)
    rospy.sleep(1)
    if retval >= 0 :
        retval = random_straight( req.max_time )
    return RandomTwistResponse(retval) 
    
    
def random_straight(max_time):
    print "-D- random_straight Server received %s"
    r = rospy.Rate(20)
    n = random.random()* max_time
    print "-I- driving for %0.3f seconds" % n 
    then = rospy.Time.now()
    dt = 0
    
    while dt < n:
        if rng_center < 0.1:
            print "-D- Range center = %d stopping!" %rng_center
            break
         
        dt_duration = rospy.Time.now() - then 
        dt = dt_duration.to_sec()
        pubStraight(.1)
        r.sleep()
    pubStraight(0) 
     
def random_turn(max_time):

    r = rospy.Rate(20)
    dir = random.randint(0,1) * 2 - 1
    n = random.random()* max_time
    print "-I- twisting for %0.3f seconds direction %d" % (n ,dir)
    
    then = rospy.Time.now()
    dt = 0
    while dt < n:
        dt_duration = rospy.Time.now() - then 
        dt = dt_duration.to_sec()
        pubTwist(dir)
        r.sleep()
    pubTwist(0) 
    return RandomTwistResponse(0) 

def random_twist_server():
    print "-D- random_twist_server"
    rospy.init_node('random_twist_server')
    s = rospy.Service('random_twist', RandomTwist, handle_random_twist)
    print "random twist server ready to twist"
    rospy.spin()
    
if __name__ == "__main__":
    random_twist_server()