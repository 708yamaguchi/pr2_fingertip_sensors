#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import PR2FingertipSensor
import rospkg
from std_srvs.srv import Empty, EmptyResponse
import yaml


class CalibratePFS(object):
    """
    Calibrate PR2FingertipSensor message (proximity, force and imu message)
    """
    def __init__(self):
        self.grippers = ['l_gripper', 'r_gripper']
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.pfs_data = {}
        for gripper in self.grippers:
            self.pfs_data[gripper] = {}
            for fingertip in self.fingertips:
                # Set dummy pfs data in case not all fingers have sensors
                self.pfs_data[gripper][fingertip] = PR2FingertipSensor()
                # Subscribe pfs data published by parse_pr2.py
                rospy.Subscriber(
                    '/pfs/{}/{}'.format(gripper, fingertip),
                    PR2FingertipSensor, self.cb, (gripper, fingertip),
                    queue_size=1)
        # Calibration
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('pr2_fingertip_sensors')
        self.pfs_params_path = package_path + '/data/pfs_params.yaml'
        # For proximity calibration
        rospy.Service('/pfs/no_object', Empty, self.no_object)
        rospy.Service('/pfs/near_object', Empty, self.near_object)
        # For force calibration
        rospy.Service('/pfs/preload', Empty, self.preload)
        rospy.Service('/pfs/dump_pfs_params', Empty, self.dump_params)

    def cb(self, msg, args):
        """
        Save PFS sensor data for calibration
        args must be tuple of (gripper, fingertip)
        """
        gripper = args[0]
        fingertip = args[1]
        self.pfs_data[gripper][fingertip] = msg

    def no_object(self, req):
        """
        Determine 'b' in I = (a / d^2) + b, and set the value to rosparam
        The service call '/pfs/no_object' must be called before '/pfs/near_object'.
        Call this service when nothing is near the PFS finger.
        """
        for gripper in self.grippers:
            for fingertip in self.fingertips:
                proximity = self.pfs_data[gripper][fingertip].proximity
                # Set dummy pfs data in case not all fingers have sensors
                if len(proximity) == 0:
                    proximity = [0] * 24
                rospy.set_param(
                    '/pfs/{}/{}/proximity_b'.format(gripper, fingertip),
                    proximity)
        rospy.loginfo("Get sensor data of 'no_object' and calibrate 'b' proximity param.")
        return EmptyResponse()

    def near_object(self, req):
        """
        Determine 'a' in I = (a / d^2) + b, and set the value to rosparam
        The service call '/pfs/near_object' must be called
        after '/pfs/no_object'.
        Call this service after wrapping the PFS finger with white conver.
        """
        for gripper in self.grippers:
            for fingertip in self.fingertips:
                proximity = self.pfs_data[gripper][fingertip].proximity
                # Set dummy pfs data in case not all fingers have sensors
                if len(proximity) == 0:
                    proximity = [0] * 24
                # Calculate 'a'
                proximity_b = rospy.get_param(
                    '/pfs/{}/{}/proximity_b'.format(gripper, fingertip))
                near_distance = 0.003  # object is now 3mm away from the sensors

                def _calc_a(prox, b):
                    a = (prox - b) * (near_distance ** 2)
                    if a <= 0:
                        rospy.logerr(
                            "Proximity param 'a' is not positive value at {} {}. Retry calibration".format(
                                gripper, fingertip))
                        a = 0  # dummy value
                    return a

                proximity_a = map(_calc_a, proximity, proximity_b)
                # Set 'a' to rosparam
                rospy.set_param(
                    '/pfs/{}/{}/proximity_a'.format(gripper, fingertip),
                    proximity_a)
        rospy.loginfo("Get sensor data of 'near_object' and calibrate 'a' proximity param.")
        return EmptyResponse()

    def preload(self, req):
        """
        Calculate preload of each force sensor
        Call this service when nothing touches the PFS finger.
        """
        for gripper in self.grippers:
            for fingertip in self.fingertips:
                force = self.pfs_data[gripper][fingertip].force
                # Set dummy pfs data in case not all fingers have sensors
                if len(force) == 0:
                    force = [0] * 24
                else:
                    if 0 in force:
                        rospy.logwarn('There is 0 force value in {} {}'.format(
                            gripper, fingertip))
                        rospy.logwarn('force value: {}'.format(
                            force))
                rospy.set_param(
                    '/pfs/{}/{}/preload'.format(gripper, fingertip),
                    force)
        rospy.loginfo("Get sensor data of 'preload' for force sensors.")
        return EmptyResponse()

    def dump_params(self, req):
        """
        Get the current pfs params and save it to data/pfs_params.yaml
        """
        params = {'pfs': {}}
        for gripper in self.grippers:
            params['pfs'][gripper] = {}
            for fingertip in self.fingertips:
                params['pfs'][gripper][fingertip] = {}
                for param_name in ['proximity_a', 'proximity_b', 'preload', 'sensitivity']:
                    param_value = rospy.get_param(
                        '/pfs/{}/{}/{}'.format(gripper, fingertip, param_name))
                    params['pfs'][gripper][fingertip][param_name] = param_value
        # Overwrite existing param file
        with open(self.pfs_params_path, "w") as f:
            yaml.dump(params, f, default_flow_style=None)
        rospy.loginfo(
            "Dump PFS sensor calibration parameters to {}.".format(self.pfs_params_path))
        rospy.loginfo(params)
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('calibrate_pfs')
    cp = CalibratePFS()
    rospy.spin()
