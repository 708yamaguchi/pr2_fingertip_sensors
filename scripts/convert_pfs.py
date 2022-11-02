#!/usr/bin/env python

from geometry_msgs.msg import WrenchStamped
import math
import rospy
from pr2_fingertip_sensors.msg import PR2FingertipSensor
from sensor_msgs.msg import Imu, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class ConvertPFS(object):
    """
    Convert PR2FingertipSensor message into proximity, force and imu message
    - proximity(sensor_msgs/PointCloud2)
    - force(geometry_msgs/WrenchStamped)
    - imu(sensor_msgs/Imu)
    """
    def __init__(self):
        self.grippers = ['l_gripper', 'r_gripper']
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.parts = [
            'pfs_a_front', 'pfs_b_top', 'pfs_b_back', 'pfs_b_left', 'pfs_b_right']
        self.pub = {}
        for gripper in self.grippers:
            self.pub[gripper] = {}
            for fingertip in self.fingertips:
                # Subscriber
                rospy.Subscriber(
                    '/pfs/{}/{}'.format(gripper, fingertip),
                    PR2FingertipSensor, self.cb, (gripper, fingertip))
                # Publisher for proximity, force and imu
                self.pub[gripper][fingertip] = {}
                for part in self.parts:
                    self.pub[gripper][fingertip][part] = {}
                    sensors = ['proximity', 'force', 'imu']
                    msg_types = [PointCloud2, WrenchStamped, Imu]
                    for sensor, msg_type in zip(sensors, msg_types):
                        if part != 'pfs_a_front' and sensor == 'imu':
                            continue
                        self.pub[gripper][fingertip][part][sensor] = rospy.Publisher(
                            '/pfs/{}/{}/{}/{}'.format(
                                gripper, fingertip, part, sensor),
                            msg_type, queue_size=1)
        self.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                       PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                       PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

    def cb(self, msg, args):
        """
        publish each sensor data
        args must be tuple of (gripper, fingertip)
        """
        gripper = args[0]
        fingertip = args[1]
        self.publish_pointcloud(msg, gripper, fingertip)
        self.publish_wrenchstamped(msg, gripper, fingertip)
        self.publish_imu(msg, gripper, fingertip)

    def sensor_num(self, part):
        """
        Args
        part: 'pfs_a_front', 'pfs_b_top', 'pfs_b_back', 'pfs_b_left' or 'pfs_b_right'

        Return
        sensor_num: The number of sensors in the specified PCB.
        """
        if part == 'pfs_a_front':
            sensor_num = 8
        else:
            sensor_num = 4
        return sensor_num

    def sensor_index(self, part, i):
        """
        Args
        part: 'pfs_a_front', 'pfs_b_top', 'pfs_b_back', 'pfs_b_left' or 'pfs_b_right'
        i: The index of sensor in each PCB. 0~7 for pfs_a_front. 0~3 for pfs_b.

        Return
        index: The index of sensor in all PCBs. This value is between 0~23.
        """
        if part == 'pfs_a_front':
            index = i
        elif part == 'pfs_b_top':
            index = 8 + i
        elif part == 'pfs_b_back':
            index = 12 + i
        elif part == 'pfs_b_left':
            index = 16 + i
        elif part == 'pfs_b_right':
            index = 20 + i
        return index

    def proximity_to_distance(self, proximity, gripper, fingertip, part, i):
        """
        Args
        proximity: The raw proximity value
        gripper: 'l_gripper' or 'r_gripper'
        fingertips: 'l_fingertip' or 'r_fingertip'
        part: 'pfs_a_front', 'pfs_b_top', 'pfs_b_back', 'pfs_b_left' or 'pfs_b_right'
        i: The index of sensor in each PCB. 0~7 for pfs_a_front. 0~3 for pfs_b.

        Return
        distance: Distance calculated from proximity value [m]
        """
        # Create method for conversion
        # I = (a / d^2) + b
        b = 100
        a = 0.01
        distance = math.sqrt(
            a / max(0.1, proximity - b))  # unit: [m]
        return distance

    def publish_pointcloud(self, msg, gripper, fingertip):
        header = Header()
        header.stamp = msg.header.stamp
        for part in self.parts:
            header.frame_id = '/' + gripper + '_' + fingertip + '_' + part
            sensor_num = self.sensor_num(part)
            points = []
            for i in range(sensor_num):
                index = self.sensor_index(part, i)  # index: 0~23
                # Convert proximity into PointCloud2
                distance = self.proximity_to_distance(
                    msg.proximity[index], gripper, fingertip, part, i)
                point = [0, 0, distance]
                points.append(point)
            prox_msg = pc2.create_cloud(header, self.fields, points)
            self.pub[gripper][fingertip][part]['proximity'].publish(prox_msg)

    def publish_wrenchstamped(self, msg, gripper, fingertip):
        header = Header()
        header.stamp = msg.header.stamp
        for part in self.parts:
            header.frame_id = '/' + gripper + '_' + fingertip + '_' + part
            sensor_num = self.sensor_num(part)
            average_force = 0.0
            for i in range(sensor_num):
                index = self.sensor_index(part, i)  # index: 0~23
                # Convert force into WrenchStamped
                average_force += msg.force[index] / float(sensor_num)
            # Publish force
            force_msg = WrenchStamped()
            force_msg.header = header
            force_scale = 0.00005  # TODO: Convert force topic value into [N]
            force_msg.wrench.force.z = average_force * force_scale
            self.pub[gripper][fingertip][part]['force'].publish(force_msg)

    def publish_imu(self, msg, gripper, fingertip):
        imu = msg.imu
        # Gyro
        gyro = imu.angular_velocity
        gyro_scale = 0.001  # TODO: Proper unit?
        msg.imu.angular_velocity.x = gyro.x * gyro_scale
        msg.imu.angular_velocity.y = gyro.y * gyro_scale
        msg.imu.angular_velocity.z = gyro.z * gyro_scale
        # Acceleration
        acc = imu.linear_acceleration
        acc_scale = 0.001  # TODO: Proper unit?
        msg.imu.linear_acceleration.x = acc.x * acc_scale
        msg.imu.linear_acceleration.y = acc.y * acc_scale
        msg.imu.linear_acceleration.z = acc.z * acc_scale
        self.pub[gripper][fingertip]['pfs_a_front']['imu'].publish(msg.imu)


if __name__ == '__main__':
    rospy.init_node('parse_pfs')
    pp = ConvertPFS()
    rospy.spin()
