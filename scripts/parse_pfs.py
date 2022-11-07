#!/usr/bin/env python

import rospy
from pr2_msgs.msg import PressureState
from pr2_fingertip_sensors.msg import PR2FingertipSensor


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
                data = self.parse(msg.l_finger_tip)
                frame_id = gripper + '_l_finger_tip_link'
            if fingertip == 'r_fingertip':
                data = self.parse(msg.r_finger_tip)
                frame_id = gripper + '_r_finger_tip_link'
            # Store parsed sensor data
            self.pfs_data[gripper][fingertip][data['packet_type']] = data
            # If all sensor data is stored, append them
            if self.pfs_data[gripper][fingertip][0] is not None \
               and self.pfs_data[gripper][fingertip][1] is not None:
                header = msg.header
                header.frame_id = frame_id
                prox = self.pfs_data[gripper][fingertip][0]['proximity'] + \
                    self.pfs_data[gripper][fingertip][1]['proximity']
                prox_ordered = self.order_data(prox)
                force = self.pfs_data[gripper][fingertip][0]['force'] + \
                    self.pfs_data[gripper][fingertip][1]['force']
                force_ordered = self.order_data(force)
                acc = self.pfs_data[gripper][fingertip][0]['imu']
                gyro = self.pfs_data[gripper][fingertip][1]['imu']
                self.publish(
                    gripper, fingertip, header,
                    prox_ordered, force_ordered, acc, gyro)
                # Reset self.pfs_data to None after publish
                self.pfs_data[gripper][fingertip][0] = None
                self.pfs_data[gripper][fingertip][1] = None

    def publish(self, gripper, fingertip, header, proximity, force, acc, gyro):
        """
        Publish parsed sensor data
        """
        pfs_msg = PR2FingertipSensor()
        pfs_msg.header = header
        pfs_msg.proximity = proximity
        pfs_msg.force = force
        pfs_msg.imu.header.stamp = header.stamp
        imu_frame_id = '/' + gripper + '_' + fingertip + '_' + 'pfs_a_front'
        pfs_msg.imu.header.frame_id = imu_frame_id
        pfs_msg.imu.linear_acceleration.x = acc[0]
        pfs_msg.imu.linear_acceleration.y = acc[1]
        pfs_msg.imu.linear_acceleration.z = acc[2]
        pfs_msg.imu.angular_velocity.x = gyro[0]
        pfs_msg.imu.angular_velocity.y = gyro[1]
        pfs_msg.imu.angular_velocity.z = gyro[2]
        self.pub[gripper][fingertip].publish(pfs_msg)

    def order_data(self, data):
        """
        Args
        data: proximity or force array (length 24)
        The order of data depends on PFS's firmware.
        Return
        data_ordered: proximity or force 'ordered' array (length 24)
        The order is 'pfs_a_front', 'pfs_b_top', 'pfs_b_back', 'pfs_b_left', 'pfs_b_right'
        """
        data_ordered = [0] * 24
        data_ordered[0:8] = data[0:8]
        data_ordered[8:12] = data[16:20]
        data_ordered[12:16] = data[20:24]
        data_ordered[16:20] = data[8:12]
        data_ordered[20:24] = data[12:16]
        return data_ordered

    def binary_to_int(self, binary_string):
        """
Args: binary string. e.g. '1010101'
return: int value. e.g. -11
"""
        uint = int(binary_string, 2)
        bits = len(binary_string)
        if (uint & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            int_val = uint - (1 << bits)        # compute negative value
        else:
            int_val = uint
        return int_val

    def parse(self, packet):
        """
Args: packet is int16[22] array
Return: dict:
          {prox[12], force[12], imu[3](acc/gyro),
           board_select, packet_type, check_sum}

c.f.
parse input
$ rosmsg show pr2_msgs/PressureState
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
int16[] l_finger_tip
int16[] r_finger_tip

parse output
$ rosmsg show pr2_fingertip_sensors/PR2FingertipSensor
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
int16[] proximity
int16[] force
sensor_msgs/Imu imu

packet protocol
https://docs.google.com/presentation/d/1VxRJWDqeDk_ryu-x1Vhj3_6BDu3gscwvNpngHKwfR4M/edit#slide=id.g1585f6b098c_0_0
        """
        # int16[] to binary
        packet_bin = ''
        for p in packet:
            # Apply mask to accept negative numbers
            packet_bin += '{0:0=16b}'.format(p & 0b1111111111111111)
        # Proximity and Force sensor
        prox = []
        force = []
        for i in range(12):
            prox_bin = packet_bin[i*24:i*24+12]
            prox_uint = int(prox_bin, 2)
            prox.append(prox_uint)
            force_bin = packet_bin[i*24+12:i*24+24]
            force_uint = int(force_bin, 2)
            force.append(force_uint)
        # imu
        imu = []
        for i in range(3):
            imu_bin = packet_bin[288+i*16:288+i*16+16]
            imu_int = self.binary_to_int(imu_bin)
            imu.append(imu_int)
        # board_select. 0: SELECT_PFS_01_SINGLE, 1: SELECT_PFS_01_ASM
        board_select_bin = packet_bin[336:340]
        board_select = self.binary_to_int(board_select_bin)
        # packet type. 0 or 1
        packet_type_bin = packet_bin[340:344]
        packet_type = self.binary_to_int(packet_type_bin)
        # check_sum (Note that check_sum is uint while others are int)
        check_sum_bin = packet_bin[344:352]
        check_sum = int(check_sum_bin, 2)
        # calculate check_sum in this node
        sum_data = 0
        for i in range(1, 42, 2):
            packet_uint8 = int(packet_bin[i*8:i*8+8], 2)
            sum_data += packet_uint8
        sum_data = sum_data % 256
        if check_sum == sum_data:
            rospy.logdebug(
                'check_sum ({}) is correctly calculated.'.format(check_sum))
        else:
            rospy.logerr(
                'check_sum ({}) is different from calculation ({}).'.format(
                    check_sum, sum_data))
        return {'proximity': prox,
                'force': force,
                'imu': imu,
                'board_select': board_select,
                'packet_type': packet_type,
                'check_sum': check_sum}


if __name__ == '__main__':
    rospy.init_node('parse_pfs')
    pp = ParsePFS()
    rospy.spin()
