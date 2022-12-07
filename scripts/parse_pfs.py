#!/usr/bin/env python

import rospy
from pr2_msgs.msg import PressureState
from pr2_fingertip_sensors.msg import PR2FingertipSensor
from pr2_fingertip_sensors import append_packets, create_pfs_msg, int16_to_binary, parse  # NOQA


class ParsePFS(object):
    """
    Get and parse two /pressure/{l or r}_gripper_motor topics.
    Concatenate them into one /pfs/{l or r}_gripper/{l or r}_finger topic.
    """
    def __init__(self):
        self.grippers = ['l_gripper', 'r_gripper']
        self.fingertips = ['l_fingertip', 'r_fingertip']
        # Publishers
        self.pub = {}
        for gripper in self.grippers:
            self.pub[gripper] = {}
            for fingertip in self.fingertips:
                self.pub[gripper][fingertip] = rospy.Publisher(
                    '/pfs/' + gripper + '/' + fingertip,
                    PR2FingertipSensor, queue_size=1)
        # PFS data to be parsed
        self.pfs_data = {}
        for gripper in self.grippers:
            self.pfs_data[gripper] = {}
            for fingertip in self.fingertips:
                self.pfs_data[gripper][fingertip] = {}
                for packet in [0, 1]:
                    self.pfs_data[gripper][fingertip][packet] = None
        # Subscribers
        # Create subscribers at the last of __init__ to avoid
        # 'object has no attribute ...' error
        rospy.Subscriber(
            "/pressure/l_gripper_motor", PressureState, self.cb, "l_gripper",
            queue_size=1)
        rospy.Subscriber(
            "/pressure/r_gripper_motor", PressureState, self.cb, "r_gripper",
            queue_size=1)

    def cb(self, msg, gripper):
        """
        Collect sensor data to self.pfs_data.
        If sensor data of all fingers are collected (all data are not None),
        publish topic
        """
        # Parse input rostopic
        for fingertip in self.fingertips:
            if fingertip == 'l_fingertip':
                packet_bin = int16_to_binary(msg.l_finger_tip)
                data = parse(packet_bin)
                frame_id = gripper + '_l_finger_tip_link'
            if fingertip == 'r_fingertip':
                packet_bin = int16_to_binary(msg.r_finger_tip)
                data = parse(packet_bin)
                frame_id = gripper + '_r_finger_tip_link'
            # Store parsed sensor data
            self.pfs_data[gripper][fingertip][data['packet_type']] = data
            # If all sensor data is stored, append them
            if self.pfs_data[gripper][fingertip][0] is not None \
               and self.pfs_data[gripper][fingertip][1] is not None:
                pfs_header = msg.header
                pfs_header.frame_id = frame_id
                imu_header = msg.header
                imu_header.frame_id = '/' + gripper + '_' + fingertip + '_' + 'pfs_a_front'
                prox_ordered, force_ordered, acc, gyro = append_packets(
                    self.pfs_data[gripper][fingertip][0],
                    self.pfs_data[gripper][fingertip][1])
                self.publish(
                    gripper, fingertip, pfs_header, imu_header,
                    prox_ordered, force_ordered, acc, gyro)
                # Reset self.pfs_data to None after publish
                self.pfs_data[gripper][fingertip][0] = None
                self.pfs_data[gripper][fingertip][1] = None

    def publish(
            self, gripper, fingertip, pfs_header, imu_header,
            proximity, force, acc, gyro):
        """
        Publish parsed sensor data
        """
        pfs_msg = create_pfs_msg(
            pfs_header, imu_header, proximity, force, acc, gyro)
        self.pub[gripper][fingertip].publish(pfs_msg)


if __name__ == '__main__':
    rospy.init_node('parse_pfs')
    pp = ParsePFS()
    rospy.spin()
