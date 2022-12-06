#!/usr/bin/env python

import serial
from serial.tools import list_ports
import argparse
import rospy
import std_msgs.msg
from pr2_fingertip_sensors.msg import PR2FingertipSensor
from pr2_fingertip_sensors import create_pfs_msg


def publish(
        publisher, gripper, fingertip, header, proximity, force, acc, gyro):
    """
    Publish parsed sensor data
    """
    pfs_msg = create_pfs_msg(
        gripper, fingertip, header, proximity, force, acc, gyro)
    publisher.publish(pfs_msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="serial port")
    parser.add_argument("--baud", "-b", default=115200, type=int, help="baud rate")
    parser.add_argument("--verbose", "-v", action='store_true')
    args = parser.parse_args()
    serial_port = args.port
    baud_rate = args.baud
    verbose = args.verbose

    key_phrase = 'serial_publish_flatten['
    serial_publish_length = 54

    rospy.init_node('serial_test')
    publisher = rospy.Publisher('/pfs/from_serial', PR2FingertipSensor, queue_size=1)

    ser = serial.Serial()
    devices = [info.device for info in list_ports.comports()]
    print("current serial port list: {}".format(devices))
    ser = serial.Serial(serial_port, baud_rate)
    print(ser)

    pfs_array = [0 for _ in range(serial_publish_length)]
    while 1:
        line = ser.readline()
        line_disp=line.strip().decode('UTF-8')
        # print(line_disp)
        if key_phrase in line_disp:
            line_without_head = line_disp.strip(key_phrase)
            line_num_split_str = line_without_head.split("]:")
            line_num = [int(i) for i in line_num_split_str]
            # print(line_num)
            if line_num[0] == 0:
                proximity = pfs_array[:24]
                force = pfs_array[24:48]
                acc = pfs_array[48:51]
                gyro = pfs_array[51:54]
                if verbose:
                    print("current pfs_array: {}".format(pfs_array))
                    print("proximity: {}".format(proximity))
                    print("force: {}".format(force))
                    print("acc: {}".format(acc))
                    print("gyro: {}".format(gyro))
                    print("-------------------------------------------------------")
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                publish(publisher, 'l_gripper', 'l_fingertip',
                        header, proximity, force, acc, gyro)
                pfs_array = [0 for _ in range(serial_publish_length)]
            pfs_array[line_num[0]] = line_num[1]
