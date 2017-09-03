#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import Twist

#the keymapping is not in use, but I do not dare to delete it for some reason
key_mapping = { 'q':[1,0,0,0,0,0], 'a':[-1,0,0,0,0,0], 'w':[0,1,0,0,0,0],
's':[0,-1,0,0,0,0], 'e':[0, 0,1,0,0,0], 'd':[0,0,-1,0,0,0],'r':[0,0,0,1,0,0], 'f':[0,0,0,-1,0,0], 't':[0,0,0,0,1,0] , 'g':[0,0,0,0,-1,0], 'z':[0,0,0,0,0,1], 'h':[0,0,0,0,0,-1]}

def keys_cb(msg, args):
  #args is a reference to the twist message that is published and it has to be updated according to msg (which is of type Joy)
 #move forward
 if msg == 'w':
  args[0].linear.x = 0.005
 #move backward
 elif msg == 's':
  args[0].linear.x = -0.005
 #move left
 if msg == 'a':
  args[0].linear.x = 0.005
 #move right
 elif msg == 'd':
  args[0].linear.x = -0.005
 #move up
 if msg == 'r':
  args[0].linear.x = 0.005
 #move down
 elif msg == 'f':
  args[0].linear.x = -0.005
 #rotate 1 +
 if msg == 'o':
  args[0].angular.x = 0.02
 #rotate 1 -
 elif msg == 'u':
  args[0].angular.x = -0.02
 #rotate 2 +
 if msg == 'l':
  args[0].angular.y = 0.02
 #rotate 2 -
 elif msg == 'j':
  args[0].angular.y = -0.02
 #rotate 3 +
 if msg == 'u':
  args[0].angular.z = 0.02;
 #rotate 3 -
 elif msg == 'j':
  args[0].angular.z = -0.02;


if __name__=='__main__':
 contTwist = [Twist()] #put empty Twist object in a list (which is filled in the callback function)
 rospy.init_node('keys2twist_continuous_node')
 joy_pub = rospy.Publisher('twist_continuous', Twist, queue_size=1)
 rospy.Subscriber('keys',String, keys_cb, contTwist)
 rate = rospy.Rate(20)
 while not rospy.is_shutdown():
  joy_pub.publish(contTwist[0])
  rate.sleep()
 rospy.spin()

