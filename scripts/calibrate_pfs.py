#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import PR2FingertipSensor


class CalibratePFS(object):
    """
    Calibrate PR2FingertipSensor message (proximity, force and imu message)
    """
    def __init__(self):
        self.grippers = ['l_gripper', 'r_gripper']
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.pub = {}
        for gripper in self.grippers:
            self.pub[gripper] = {}
            for fingertip in self.fingertips:
                # Subscribe raw data
                rospy.Subscriber(
                    '/pfs/{}/{}/raw'.format(gripper, fingertip),
                    PR2FingertipSensor, self.cb, (gripper, fingertip))
                # Publish calibrated data
                self.pub[gripper][fingertip] = rospy.Publisher(
                    '/pfs/{}/{}'.format(gripper, fingertip),
                    PR2FingertipSensor, queue_size=1)

    def cb(self, msg, args):
        """
        Publish calibrated PFS sensor data
        args must be tuple of (gripper, fingertip)
        """
        gripper = args[0]
        fingertip = args[1]
        msg_calibrated = self.calibrate(msg)
        self.pub[gripper][fingertip].publish(msg_calibrated)

    def calibrate(self, msg):
        """
        Calibrate sensor data
        """
        # Need calibration for proximity and force sensors
        return msg


if __name__ == '__main__':
    rospy.init_node('calibrate_pfs')
    cp = CalibratePFS()
    rospy.spin()
