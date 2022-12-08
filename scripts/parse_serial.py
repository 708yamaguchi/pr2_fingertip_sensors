#!/usr/bin/env python

import serial
from serial.tools import list_ports
import argparse
import rospy
from std_msgs.msg import Header
from pr2_fingertip_sensors.msg import PR2FingertipSensor
from pr2_fingertip_sensors import append_packets, binary_to_int, create_pfs_msg, parse  # NOQA


class ParseSerial(object):
    """
    Get and parse serial data from PFS.
    Concatenate two packets into one topic.
    """

    def __init__(self, serial_port, baud_rate, verbose):
        self.pub = rospy.Publisher(
            '/pfs/from_serial', PR2FingertipSensor, queue_size=1)
        self.ser = serial.Serial() ## for detect ports
        devices = [info.device for info in list_ports.comports()]
        rospy.loginfo("current serial port list: {}".format(devices))
        self.ser = serial.Serial(serial_port, baud_rate)
        rospy.loginfo(self.ser)
        self.packet_bytes = 44  # each packet is 44 bytes
        self.pfs_data = [None, None]

    def publish(self, header, proximity, force, acc, gyro):
        """
        Publish parsed sensor data
        """
        pfs_msg = create_pfs_msg(
            header, header, proximity, force, acc, gyro)
        self.pub.publish(pfs_msg)

    def parse_serial(self):
        # Get packet data. The delimiter is newline.
        line = self.ser.readline()
        line_disp = line.rstrip()
        res_disp = ''.join(format(ord(i), '08b') for i in line_disp)
        if (len(res_disp) != self.packet_bytes * 8):
            return False
        # Parse packet and get sensor data.
        packet = parse(res_disp)
        self.pfs_data[packet['packet_type']] = packet
        # Append two packets into one PFS data
        if self.pfs_data[0] != None and self.pfs_data[1] != None:
            prox_ordered, force_ordered, acc, gyro = append_packets(
                self.pfs_data[0],
                self.pfs_data[1])
            header = Header()
            header.stamp = rospy.Time.now()
            self.publish(
                header, prox_ordered, force_ordered, acc, gyro)
            self.pfs_data[0] = None
            self.pfs_data[1] = None
        return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port", "-p", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument(
        "--baud", "-b", default=57600, type=int, help="baud rate")
    parser.add_argument(
        "--verbose", "-v", action='store_true')
    args = parser.parse_args()
    serial_port = args.port
    baud_rate = args.baud
    verbose = args.verbose

    rospy.init_node('parse_serial')
    ps = ParseSerial(serial_port, baud_rate, verbose)
    rate = rospy.Rate(100)  # 100Hz
    while not rospy.is_shutdown():
        ps.parse_serial()
        rate.sleep()
