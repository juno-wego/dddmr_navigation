# Go2 Mapping / Save / Navigation Commands

Go2 실기에서 맵핑하고, 맵을 저장하고, 저장한 맵을 불러와 자율주행하는 명령어만 정리한 문서입니다.

## 1. 공통 준비

새 터미널마다:

```bash
cd ~/dddmr_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 맵핑 시작

기본 실행:

```bash
ros2 launch go2_mapping go2_mapping.launch.py
```

인자까지 명시:

```bash
ros2 launch go2_mapping go2_mapping.launch.py \
  interface:=enp88s0 \
  aurora_ip:=192.168.11.1 \
  use_go2:=true \
  use_aurora:=true \
  use_mid360:=true \
  use_go2_camera:=true \
  rviz:=true \
  raw_lidar_topic:=/merged/points \
  lidar_topic:=/go2/lidar_points_base \
  odom_topic:=/slamware_ros_sdk_server_node/odom
```

맵핑 결과는 저장 전까지 메모리에 있고, 저장하면 아래 경로에 생깁니다.

```text
~/dddmr_navigation/maps/go2/<timestamp>/
```

## 3. 맵 저장

맵핑 launch를 켜둔 상태에서 새 터미널:

```bash
cd ~/dddmr_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run go2_mapping save_go2_map.py
```

저장 루트 직접 지정:

```bash
ros2 run go2_mapping save_go2_map.py \
  --map-root ~/dddmr_navigation/maps/go2
```

최근 저장 맵 확인:

```bash
ls -td ~/dddmr_navigation/maps/go2/* | head
```

## 4. 저장한 맵 불러오기 / 자율주행

최신 저장 맵 자동 사용:

```bash
ros2 launch go2_mapping go2_navigation.launch.py
```

특정 맵 지정:

```bash
ros2 launch go2_mapping go2_navigation.launch.py \
  map_dir:=~/dddmr_navigation/maps/go2/<timestamp>
```

인자까지 명시:

```bash
ros2 launch go2_mapping go2_navigation.launch.py \
  map_dir:=~/dddmr_navigation/maps/go2/<timestamp> \
  map_root:=~/dddmr_navigation/maps/go2 \
  interface:=enp88s0 \
  aurora_ip:=192.168.11.1 \
  use_go2:=true \
  use_aurora:=true \
  use_mid360:=true \
  use_go2_camera:=true \
  rviz:=true \
  raw_lidar_topic:=/merged/points \
  lidar_topic:=/go2/lidar_points_base \
  odom_topic:=/slamware_ros_sdk_server_node/odom
```

## 5. 인자 정리

### `go2_mapping.launch.py`

| 인자 | 기본값 |
|---|---|
| `interface` | `enp88s0` |
| `aurora_ip` | `192.168.11.1` |
| `use_go2` | `true` |
| `use_aurora` | `true` |
| `use_mid360` | `true` |
| `use_go2_camera` | `true` |
| `rviz` | `true` |
| `raw_lidar_topic` | `/merged/points` |
| `lidar_topic` | `/go2/lidar_points_base` |
| `odom_topic` | `/slamware_ros_sdk_server_node/odom` |

### `save_go2_map.py`

| 인자 | 기본값 |
|---|---|
| `--map-root` | `$DDDMR_MAPPING_DIR` 또는 `~/dddmr_navigation/maps/go2` |

### `go2_navigation.launch.py`

| 인자 | 기본값 |
|---|---|
| `map_dir` | 빈 값. 비우면 `map_root` 아래 최신 맵 사용 |
| `map_root` | `~/dddmr_navigation/maps/go2` |
| `interface` | `enp88s0` |
| `aurora_ip` | `192.168.11.1` |
| `use_go2` | `true` |
| `use_aurora` | `true` |
| `use_mid360` | `true` |
| `use_go2_camera` | `true` |
| `rviz` | `true` |
| `raw_lidar_topic` | `/merged/points` |
| `lidar_topic` | `/go2/lidar_points_base` |
| `odom_topic` | `/slamware_ros_sdk_server_node/odom` |
