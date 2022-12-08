from pr2_fingertip_sensors.msg import PR2FingertipSensor


def create_pfs_msg(
        pfs_header, imu_header, proximity, force, acc, gyro):
    """
    Create pr2_fingertip_sensors/PR2Fingertipsensor message

    Args
    pfs_header: std_msgs/Header object for whole PFS finger
    imu_header: std_msgs/Header object for IMU on PFS A board
    proximity: List of int16. Length is 24.
    force: List of int16. Length is 24.
    acc: List of float. Length is 3.
    gyro: List of float. Length is 3.

    Return
    pr2_fingertip_sensors/PR2Fingertipsensor object
    """
    pfs_msg = PR2FingertipSensor()
    pfs_msg.header = pfs_header
    pfs_msg.proximity = proximity
    pfs_msg.force = force
    pfs_msg.imu.header = imu_header
    pfs_msg.imu.linear_acceleration.x = acc[0]
    pfs_msg.imu.linear_acceleration.y = acc[1]
    pfs_msg.imu.linear_acceleration.z = acc[2]
    pfs_msg.imu.angular_velocity.x = gyro[0]
    pfs_msg.imu.angular_velocity.y = gyro[1]
    pfs_msg.imu.angular_velocity.z = gyro[2]
    return pfs_msg


def order_data(data):
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


def binary_to_int(binary_string):
    """
Args: binary string. e.g. '1010101'
return: int value. e.g. -11
"""
    uint = int(binary_string, 2)
    bits = len(binary_string)
    if (uint & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        int_val = uint - (1 << bits)        # compute negative value
    else:
        int_val = uint
    return int_val


def int16_to_binary(packet):
    # int16[] to binary
    packet_bin = ''
    for p in packet:
        # Apply mask to accept negative numbers
        packet_bin += '{0:0=16b}'.format(p & 0b1111111111111111)
    return packet_bin


def parse(packet_bin):
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
\
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
        imu_int = binary_to_int(imu_bin)
        imu.append(imu_int)
    # board_select. 0: SELECT_PFS_01_SINGLE, 1: SELECT_PFS_01_ASM
    board_select_bin = packet_bin[336:340]
    board_select = binary_to_int(board_select_bin)
    # packet type. 0 or 1
    packet_type_bin = packet_bin[340:344]
    packet_type = binary_to_int(packet_type_bin)
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
        pass
        # print('check_sum ({}) is correctly calculated.'.format(check_sum))
        # rospy.logdebug(
        #     'check_sum ({}) is correctly calculated.'.format(check_sum))
    else:
        pass
        # print('check_sum ({}) is different from calculation ({}).'.format(
        #     check_sum, sum_data))
        # rospy.logdebug(
        #     'check_sum ({}) is different from calculation ({}).'.format(
        #         check_sum, sum_data))
    return {'proximity': prox,
            'force': force,
            'imu': imu,
            'board_select': board_select,
            'packet_type': packet_type,
            'check_sum': check_sum}


def append_packets(packet1, packet2):
    """
    Append two packets defined for PFS sensor data.
    https://docs.google.com/presentation/d/1VxRJWDqeDk_ryu-x1Vhj3_6BDu3gscwvNpngHKwfR4M/edit#slide=id.g1585f6b098c_0_0

    Args
    packet1: Dict of packet type 1 message.
    packet2: Dict of packet type 1 message.

    Return
    proximity_ordered: List of int16. Length is 24.
    force_ordered: List of int16. Length is 24.
    acc: List of float. Length is 3.
    gyro: List of float. Length is 3.
    """
    proximity = packet1['proximity'] + packet2['proximity']
    proximity_orderd = order_data(proximity)
    force = packet1['force'] + packet2['force']
    force_ordered = order_data(force)
    gyro = packet1['imu']
    acc = packet2['imu']
    return proximity_orderd, force_ordered, acc, gyro
