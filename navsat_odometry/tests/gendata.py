#!/usr/bin/python
#
#  Navigation satellite odometry test data generator script.
#
#
# $Id$ 

PKG_NAME = 'navsat_odometry'

# ROS node setup
import roslib;
roslib.load_manifest(PKG_NAME)
import rospy

# ROS messages
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

def test(hz):
    "run test at specified frequency"

    gps_pub = rospy.Publisher('gps', NavSatFix)
    imu_pub = rospy.Publisher('imu', Imu)

    # set position at University of Texas, Austin, Pickle Research Campus
    fix = NavSatFix()
    fix.latitude = 30.385315
    fix.longitude = -97.728524
    fix.altitude = 209.0
    fix.header.frame_id = "/odom"

    # set IMU not moving
    imu = Imu()
    imu.header.frame_id = "/base_link"

    while not rospy.is_shutdown():

        fix.header.stamp = rospy.Time.now()
        gps_pub.publish(fix)
        imu.header.stamp = fix.header.stamp
        imu_pub.publish(imu)

        rospy.sleep(1.0/hz)

if __name__ == '__main__':
    rospy.init_node('gendata')
    rospy.loginfo('starting navsat_odometry test')
    try:
        test(20.0)
    except rospy.ROSInterruptException: pass

    rospy.loginfo('navsat_odometry test completed')
