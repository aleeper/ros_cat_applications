#!/usr/bin/env python

import roslib
roslib.load_manifest("cat_user_entity")

import rospy
from math import *
import tf
import numpy as np
from tf import transformations
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped

class Calibrator:
    def __init__(self):
        self.sub = rospy.Subscriber("ft/l_gripper_motor", WrenchStamped, self.callback, queue_size = 2)
        self.pub = rospy.Publisher("output", WrenchStamped)
        self.tfl = tf.TransformListener()
        self.m = 2.5
        self.g = 9.81
        self.d = 0.08

    def callback(self, msg):
        #rospy.loginfo(rospy.get_name() + ": I heard %s" % str(msg))
        self.tfl.waitForTransform("base_link", "l_force_torque_link", msg.header.stamp, rospy.Duration(1.0))
        trans, rot = self.tfl.lookupTransform("base_link", "l_force_torque_link", msg.header.stamp) 
        mat = transformations.quaternion_matrix(rot)
        R = mat[:3,:3]/mat[3,3]
        f_in = np.array( [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z] )
        t_in = np.array( [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z] )
        
        f_est = self.m * self.g * R[2,:]
        t_est = self.m * self.g * self.d * np.array( [0, R[2,2], -R[2,1]] )
       
        #print "force est: ", f_est 
        #print "torque est: ", t_est 
        f_out = f_in - f_est
        t_out = t_in - t_est
       
        out = WrenchStamped()
        out.header = msg.header 
        out.wrench.force  = Vector3(*f_out)
        out.wrench.torque = Vector3(*t_out)
        self.pub.publish(out)



if __name__ == '__main__':
  rospy.init_node('FT_calibrator', anonymous=True)
  c = Calibrator()
  rospy.spin()
