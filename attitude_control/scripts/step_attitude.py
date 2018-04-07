#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

class step_attitude(object):
    def __init__(self):
        rospy.init_node('step_attitude')
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.imu_set = False
        self.imu_q = None

        self.step_size = 10.0 # deg
        self.thrust = 0.0

        self.setpoint_pub = rospy.Publisher('/att_control/attitude_target', AttitudeTarget, queue_size=10)

    def imu_cb(self, data):
        q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        if self.imu_set:
            q = self.imu_q

        euler = euler_from_quaternion(q)
        q_step = quaternion_from_euler(0.0, 0.0, np.deg2rad(self.step_size))

        new_q = quaternion_multiply(q, q_step)
        new_euler = euler_from_quaternion(new_q)

        self.imu_set = True
        self.imu_q = q
        print "Current Yaw: " + str(np.rad2deg(euler[2]))
        print "New Yaw: " + str(np.rad2deg(new_euler[2]))

        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.thrust = self.thrust
        msg.orientation.x = new_q[0]
        msg.orientation.y = new_q[1]
        msg.orientation.z = new_q[2]
        msg.orientation.w = new_q[3]

        self.setpoint_pub.publish(msg)

if __name__ == '__main__':
    node = step_attitude()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
