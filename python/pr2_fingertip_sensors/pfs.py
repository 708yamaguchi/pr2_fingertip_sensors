from pr2_fingertip_sensors.msg import PR2FingertipSensor


def create_pfs_msg(
        gripper, fingertip, header, proximity, force, acc, gyro):
    """
    Create pr2_fingertip_sensors/PR2Fingertipsensor message

    Args
    gripper: String object. 'l_gripper' or 'r_gripper'
    fingertip: String object. 'l_fingertip' or 'r_fingertip'
    header: std_msgs/Header object
    proximity: List of int16. Length is 24.
    force: List of int16. Length is 24.
    acc: List of float. Length is 3.
    gyro: List of float. Length is 3.

    Return
    pr2_fingertip_sensors/PR2Fingertipsensor object
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
    return pfs_msg
