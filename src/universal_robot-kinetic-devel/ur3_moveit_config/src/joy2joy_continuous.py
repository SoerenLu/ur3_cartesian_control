#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

key_mapping = { 'q':[1,0,0,0,0,0], 'a':[-1,0,0,0,0,0], 'w':[0,1,0,0,0,0],
's':[0,-1,0,0,0,0], 'e':[0, 0,1,0,0,0], 'd':[0,0,-1,0,0,0],'r':[0,0,0,1,0,0], 'f':[0,0,0,-1,0,0], 't':[0,0,0,0,1,0] , 'g':[0,0,0,0,-1,0], 'z':[0,0,0,0,0,1], 'h':[0,0,0,0,0,-1]}

def keys_cb(msg, args):
  #args is a reference to the last recorded Joy message and has to be updated
 args[0] = msg

if __name__=='__main__':
 currJoy = [Joy()]
 rospy.init_node('keys_to_jointDeltas')
 joy_pub = rospy.Publisher('joy_continuous', Joy, queue_size=1)
 rospy.Subscriber('joy',Joy, keys_cb, currJoy)
 rate = rospy.Rate(20)
 while not rospy.is_shutdown():
  joy_pub.publish(currJoy[0])
  rate.sleep()
 rospy.spin()
