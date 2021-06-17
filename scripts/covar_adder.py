#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np

odom_pub = rospy.Publisher("/encoder_odom_cov", Odometry, queue_size=10)
imu_pub = rospy.Publisher("/imu/data_cov", Imu, queue_size=10)


def odom_callback(odom_msg):
    odom_msg.twist.covariance = np.diagflat([0.2, 0.2, 0.0, 0.0, 0.0, 1.00]).flatten()
    odom_pub.publish(odom_msg)


def imu_callback(imu_msg):
    imu_msg.orientation_covariance = np.diagflat([0.01, 0.01, 0.3]).flatten()
    imu_msg.angular_velocity_covariance = np.diagflat([0.01, 0.01, 0.3]).flatten()
    imu_msg.linear_acceleration_covariance = np.diagflat([0.01, 0.01, 0.01]).flatten()
    imu_pub.publish(imu_msg)


if __name__ == '__main__':
    rospy.init_node('covar_adder', anonymous=True)

    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/encoder_odom", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
