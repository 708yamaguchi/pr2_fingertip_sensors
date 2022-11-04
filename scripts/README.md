# ROS nodes

Here, to avoid duplicate expressions, multiple topic names are summarized by abbreviating them as follows:
  - `{l_r}`: `l` or `r`
  - `{part}`: `pfs_a_front`, `pfs_b_top`, `pfs_b_back`, `pfs_b_left` or `pfs_b_right`
  - `{sensor_num}`: the number of sensors on the each PFS board

## parse_pfs.py

  This node parses PFS sensor data by combining two consecutive topics.

### Subscribe topics

  ROS topics output from PR2 gripper MCB.

  - `/pressure/{l_r}_gripper_motor` (`pr2_msgs/PressureState`)

### Publish topics

  Parsed PFS topics.

  - `/pfs/{l_r}_gripper/{l_r}_fingertip` [`pr2_fingertip_sensors/PR2FingertipSensor`]

## convert_pfs.py

  This node converts parsed PFS data into ROS topics decomposed for each sensor.
  This node publishes
  - force data for each PFS board (6 axis, unit: [N])
  - imu data for each fingertip (6 axis, unit: [rad/s], [m/s^2])
  - proximity data for each sensor (Pointcloud considering the position of each sensor and the distance calculated from the sensor. `I = (a/d^2) + b`)

### Subscribe topics

  Parsed PFS topics.

  - `/pfs/{l_r}_gripper/{l_r}_fingertip` [`pr2_fingertip_sensors/PR2FingertipSensor`]

### Publish topics

  ROS topics decomposed for each sensor.
  The last proximity topics are for RViz visualization.

  - `/pfs/{l_r}_gripper/{l_r}_fingertip/{part}/force` [`geometry_msgs/WrenchStamped`]
  - `/pfs/{l_r}_gripper/{l_r}_fingertip/{part}/imu` [`sensor_msgs/Imu`]
  - `/pfs/{l_r}_gripper/{l_r}_fingertip/{part}/proximity` [`sensor_msgs/PointCloud2`]
  - `/pfs/{l_r}_gripper/{l_r}_fingertip/{part}/proximity/{sensor_num}` [`sensor_msgs/PointCloud2`]

### Parameters

  Parameters used to convert PFS sensor data. The default value is saved at `pr2_fingertip_sensors/data/pfs_params.yaml`

  - `/pfs/{l_r}_gripper/{l_r}_fingertip/force_scale` [float list, 24 elements]
  - `/pfs/{l_r}_gripper/{l_r}_fingertip/preload` [float list, 24 elements]
  - `/pfs/{l_r}_gripper/{l_r}_fingertip/proximity_a` [float list, 24 elements]
  - `/pfs/{l_r}_gripper/{l_r}_fingertip/proximity_b` [float list, 24 elements]

## calibrate_pfs.py

  This ROS node is used only when calibrating each sensor. Calibrate each sensor by calling ROS service.

### Subscribe topics

  Parsed PFS topics.

  - `/pfs/{l_r}_gripper/{l_r}_fingertip` [`pr2_fingertip_sensors/PR2FingertipSensor`]

### Publish topics

None

### Advertise services

#### Calibrate proximity sensor

  Calibrate proximity parameters 'b' in I = (a / d^2) + b. Run the following command when nothing is near the PFS finger.

  - `/pfs/no_object` [`std_srvs/Empty`]

  Calibrate proximity parameters 'a' in I = (a / d^2) + b. Run the following command after wrapping the PFS finger with white conver. This command must be called after the above command.

  - `/pfs/near_object` [`std_srvs/Empty`]

#### Calibrate force sensor

  Calibrate force sensor preload. Run the following command when nothing touches the PFS finger.

  - `/pfs/preload` [`std_srvs/Empty`]

#### Dump calibration params

  Dump calibration parameters to yaml file (`pr2_fingertip_sensors/data/pfs_params.yaml`).