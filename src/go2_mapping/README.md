# go2_mapping

`go2_mapping` is a thin ROS 2 launch package that starts:

- `go2_bringup` sensors (`go2`, `aurora`, `livox`)
- `lego_loam_bor` online mapping
- RViz with the existing LeGO-LOAM mapping view

## Launch

```bash
ros2 launch go2_mapping go2_mapping.launch.py
```

Useful arguments:

```bash
ros2 launch go2_mapping go2_mapping.launch.py odom_source:=go2 lidar_source:=utlidar
ros2 launch go2_mapping go2_mapping.launch.py odom_source:=aurora lidar_source:=utlidar
ros2 launch go2_mapping go2_mapping.launch.py odom_source:=go2 lidar_source:=livox
```

## Bag Launch

```bash
ros2 launch go2_mapping go2_mapping_bag.launch.py bag_file_dir:=/path/to/bag
```

Useful arguments:

```bash
ros2 launch go2_mapping go2_mapping_bag.launch.py bag_file_dir:=/path/to/bag odom_source:=aurora lidar_source:=utlidar
ros2 launch go2_mapping go2_mapping_bag.launch.py bag_file_dir:=/path/to/bag odom_source:=go2 lidar_source:=utlidar
ros2 launch go2_mapping go2_mapping_bag.launch.py bag_file_dir:=/path/to/bag odom_source:=go2 lidar_source:=livox
```

## Notes

- `lego_loam_bor` currently subscribes to one point cloud topic, so this package selects either `utlidar` or `livox` as the mapping input.
- The default path is `utlidar + wheel/aurora odometry`, which is the safest choice for ground tile generation.
- `livox` mode is provided as an alternate configuration, but it is not a fused dual-lidar pipeline.
