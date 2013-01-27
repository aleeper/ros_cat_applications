#!/usr/bin/env python

import roslib
roslib.load_manifest("cat_user_entity")

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
import argparse

FT_TOPIC = "pr2_netft_zeroer/wrench_zeroed"
ERROR_TOPIC = "cat_backend_node/metrics/tracking_error"

class Writer:
    def __init__(self, name):
        self.ft_sub = rospy.Subscriber(FT_TOPIC, WrenchStamped, self.ft_callback)
        self.error_sub = rospy.Subscriber(ERROR_TOPIC, Float64MultiArray, self.error_callback)
        self.last_data = {"force_mag": 0, "torque_mag": 0, "distance": 0, "angle": 0 }
        self.datafile = open('data/%s'%name, 'w')
        self.writeHeader()
        self.timer = rospy.Timer(rospy.Duration(0.5), self.writeLine)
        
    def writeHeader(self):
        header = "{:10} {:10} {:10} {:10} {:10}\n".format("Time", "force_mag", "torque_mag", "distance", "angle")
        self.datafile.write(header)
        rospy.loginfo(header)


    def writeLine(self, event):
        #
        line = "{:10} {:10} {:10} {:10} {:10}\n".format(event.current_real.to_sec(), 
                                                        self.last_data["force_mag"],
                                                        self.last_data["torque_mag"],
                                                        self.last_data["distance"],
                                                        self.last_data["angle"]
                                                       )
        
        self.datafile.write(line)
        rospy.loginfo(line)

    def vector3_to_array(self, vec):
        return np.array( [vec.x, vec.y, vec.z] )
    
    def ft_callback(self, msg):
        force = vector3_to_array(msg.wrench.force)
        torque = vector3_to_array(msg.wrench.torque)
        
        force_mag = np.linalg.norm(force)
        torque_mag = np.linalg.norm(torque)

        self.last_data["force_mag"] = force_mag
        self.last_data["torque_mag"] = torque_mag

    def error_callback(self, msg):
        distance = msg.data[0]
        angle = msg.data[1]

        self.last_data["distance"] = distance
        self.last_data["angle"] = angle


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Collect some data.')
  parser.add_argument('--name', '-n', help='The file name', default='junk_data')
  args = parser.parse_args()
  rospy.loginfo("Initializing writer node, writing to file {}".format(args.name)) 
  rospy.init_node('data_writer_node', anonymous=True) 
  c = Writer(args.name)
  rospy.spin()
