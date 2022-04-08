# wit_node

This is the non-official ROS2 package for wit motion company imu and gps sensor. 

Tested with WitMotion WT901C RS232.


## Dependencies and Install

1. ROS2 foxy
2. ros-foxy-ecl-*

Install ecl by

```bash
distro="foxy"
sudo apt install ros-${distro}-ecl-devices \
                 ros-${distro}-ecl-sigslots \
                 ros-${distro}-ecl-time \
                 ros-${distro}-ecl-build \
                 ros-${distro}-ecl-geometry
```

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/fateshelled/wit_node
cd ~/ros2_ws
colcon build --symlink-install
```



## Usage

```bash
sudo chmod 666 /dev/ttyUSB0

ros2 launch wit_node wit.launch.py

# launch with rviz2
ros2 launch wit_node wit_visualize.launch.py
```

About parameter:

| name | default | type | about |
| - | - | - | - |
| port       | /dev/ttyUSB0 | string  | device name in Linux system. |
| baud_rate  | 115200       | int     | speed of communication with wit motion device (bps). |
| frame_id   | /imu_link    | string  | publishing topic frame_id. |
| publish_hz | 10.0         | float64 | publishing rate (Hz). |


## Msg

### ImuGpsRaw

> Header header
>
> float64 time
>
> float64[] acc
>
> float64[] gyro
>
> float64[] rpy
>
> float64[] mag
>
> uint16[]  ps #port state
>
> float64   temperature
>
> float64   altitude
>
> float64   ap #atmosphere pressure
>
> float64   latitude
>
> float64   longtitude
>
> float64   gpsh #gps height
>
> float64   gpsy #gps yaw
>
> float64   gpsv #gps velocity
>
> float64[] quarternion
>
> uint8     sn #satelites number
>
> float64[] dop

## Published Topics

### /imu (sensor_msgs/Imu)

The standard ROS imu sensor msg which include orientation by filtered RPY.

### /gps (sensor_msgs/NavSatFix)

The standard ROS gps or navigation satellites msg.

### /wit/raw_data (wit_node/ImuGpsRaw)

All raw data provided by the wit device, including nine axises data, atmosphere pressure, temperature, latitude,longitude, altitude, satellites number .etc

### /wit/related_yaw (std_msgs/Float64)

The offseted imu yaw data, which means the zero direction is start direction.

### /wit/imu_pose (geometry_msgs::msg::PoseStamped)

Pose msg for visualize imu sensor msg with rviz2.



## Subscribed Topics

### /wit/reset_offset (std_msgs/Empty)

Reset the offset yaw angle to current yaw,  so the zero direction is turned to current direction.



