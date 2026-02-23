# HDMapping-LIO

ROS2 wrapper for [HDMapping](https://github.com/MapsHD/HDMapping) LiDAR Inertial Odometry (LIO).

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src/ && git clone --recursive https://github.com/MapsHD/HDMapping-LIO
cd ~/ros2_ws/ && rosdep install --from-paths src --ignore-src -y
colcon build --packages-select hdmapping_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch hdmapping_lio hdmapping_lio.launch.py
```

## Usage

```bash
ros2 launch hdmapping_lio hdmapping_lio.launch.py \
    imu_topic:=/livox/imu \
    lidar_topic:=/livox/lidar \
    use_rviz:=true
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `params_file` | `default_params.yaml` | Path to parameter YAML file |
| `imu_topic` | `/imu` | IMU topic name |
| `lidar_topic` | `/points` | LiDAR point cloud topic name |
| `use_rviz` | `false` | Launch RViz2 |
| `rviz_config` | `hdmapping_lio.rviz` | RViz config file |

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/imu` | `sensor_msgs/msg/Imu` | IMU (accelerometer + gyroscope) |
| `/points` | `sensor_msgs/msg/PointCloud2` | LiDAR point cloud |

### Published

| Topic | Type | Description |
|---|---|---|
| `hdmapping/odom` | `nav_msgs/msg/Odometry` | Current pose and velocity |
| `hdmapping/trajectory` | `nav_msgs/msg/Path` | Full trajectory history |
| `hdmapping/map` | `sensor_msgs/msg/PointCloud2` | Accumulated map point cloud |

## License

MIT
