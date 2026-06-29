# DDDMR Go2 Navigation Summary

## 1. Mapping

현재 Go2의 mapping은 LeGO-LOAM 계열 파이프라인을 기반으로 한다.

기본 흐름은 다음과 같다.

```text
LiDAR point cloud
-> ImageProjection
-> FeatureAssociation
-> MapOptimization
-> map / ground.pcd / poses.pcd 저장
```

기존 LeGO-LOAM 방식은 LiDAR의 `ring` 구조를 이용해 point cloud를 range image로 만들고, 인접 ring 간 기울기와 거리 차이를 이용해 ground를 판단한다.

하지만 현재 Go2 실기 구성에서는 전면 `utlidar`와 추가 LiDAR 데이터가 기존 LeGO-LOAM이 가정하는 정형화된 ring 구조와 완전히 맞지 않는다. 그래서 현재 방식은 point cloud를 로봇 기준 좌표계인 `base_footprint`로 변환한 뒤, z값을 이용해 ground를 추정한다.

요약하면 다음과 같다.

```text
기존 방식:
ring 간 slope / dz / ground_fov 기준으로 ground 판단

현재 방식:
base_footprint 기준 z값이 일정 범위 안이면 ground로 판단
```

즉 전체 SLAM 파이프라인을 바꾼 것이 아니라, `ImageProjection` 단계의 ground extraction 방식을 Go2 실기 환경에 맞게 수정한 것이다.

## 2. Base Footprint 기반 Ground 추정

`base_footprint`는 로봇의 바닥 중심 좌표계이다.

```text
x: 로봇 전방
y: 로봇 좌측
z: 위쪽
```

센서마다 자기 좌표계가 다르기 때문에, 센서 기준 point cloud를 그대로 사용하면 같은 지면도 센서 장착 위치에 따라 서로 다른 z값을 가진다.

따라서 Go2에서는 센서 데이터를 먼저 `base_footprint` 기준으로 변환한다. 이 기준에서는 로봇이 서 있는 지면이 보통 `z = 0` 근처에 위치한다.

```text
센서 기준:
지면 z = -0.3 또는 -0.7 등 센서 장착 높이에 따라 달라짐

base_footprint 기준:
지면 z ~= 0
```

이후 일정 z 범위 안에 있는 point를 ground 후보로 분류한다.

예시:

```text
base_footprint 기준 z = -0.20 ~ 0.20
-> ground 후보
```

이 방식은 LiDAR의 ring 구조에 덜 의존하고, 실제 Go2 장착 위치와 주행 기준에 맞춰 ground를 더 단순하고 안정적으로 추정할 수 있다.

## 3. 경사로 상황에서의 의미

여기서 말하는 `z = 0`은 map/world 기준 절대 높이가 아니라, 로봇의 현재 `base_footprint` 기준 상대 높이이다.

로봇이 언덕을 올라가면 map 기준 z는 증가할 수 있다. 하지만 로봇 기준 좌표계도 함께 이동하므로, 로봇 주변 지면은 여전히 `base_footprint` 기준으로 0 근처에 위치한다.

```text
map 기준:
로봇이 언덕 위로 올라가면 z 증가

base_footprint 기준:
로봇 주변 지면은 여전히 z ~= 0
```

다만 급경사나 멀리 있는 지면에서는 z 편차가 커질 수 있다. 그래서 현재 방식은 완전한 지형 모델링이라기보다, 로봇 주변 근거리 ground를 안정적으로 잡기 위한 단순화된 기준이다.

## 4. Localization

기존 localization은 ring 기반 LeGO-LOAM 계열 3D feature localization으로 볼 수 있다.

기존 방식은 다음과 같은 전제를 가진다.

```text
하나의 회전형 LiDAR
규칙적인 vertical ring
센서 프레임 기준 organized point cloud
```

기존 흐름:

```text
LiDAR point cloud
-> range image 생성
-> ring 기반 ground / edge / surface feature 추출
-> 현재 feature와 저장된 map feature 정합
-> robot pose 추정
-> map -> odom TF 출력
```

현재 Go2 방식은 실제 센서 구성에 맞춰 point cloud를 `base_footprint` 기준으로 통일하고, ground를 z-height 기반으로 분리한다.

현재 흐름:

```text
Go2 LiDAR / MID360 point cloud
-> base_footprint 기준으로 변환 및 병합
-> z값 기반 ground 분리
-> sharp / flat feature 추출
-> map과 현재 feature 정합
-> map -> odom TF 출력
```

즉 기존 방식은 센서 scan 구조에 강하게 의존하고, 현재 방식은 로봇 기준 좌표계와 실제 ground 높이를 더 직접적으로 활용한다.

## 5. Localization Framework 그림 설명

그림의 흐름은 LiDAR 기반 3D localization 구조를 나타낸다.

```text
LiDAR
-> MCL-feature
-> MCL_3DL
-> map -> odom TF
```

LiDAR에서 들어온 `sensor_msgs/PointCloud2`를 `MCL-feature`가 처리한다. 여기서 원본 point cloud를 localization에 쓰기 좋은 두 종류의 feature로 나눈다.

```text
sharp feature:
모서리, 벽 경계처럼 변화가 큰 점

flat feature:
바닥, 벽면처럼 평평한 면을 이루는 점
```

이 feature들이 `MCL_3DL`로 들어가고, `MCL_3DL`은 여기에 wheel odometry와 RViz에서 입력한 initial pose를 함께 사용해 로봇의 현재 위치를 추정한다.

최종적으로 `MCL_3DL`은 `map -> odom` TF를 publish한다. 이 TF가 생기면 ROS 전체 시스템은 현재 odom 기준 위치가 map 상에서 어디인지 알 수 있다.

발표용 문장:

> LiDAR point cloud에서 edge 계열의 sharp feature와 plane 계열의 flat feature를 추출하고, 이를 wheel odometry 및 초기 위치와 함께 MCL_3DL에 입력한다. MCL_3DL은 저장된 map과 현재 feature를 정합해 로봇 위치를 추정하고, 최종적으로 `map -> odom` TF를 출력한다.

## 6. dddmr_perception_3d

`dddmr_perception_3d`는 localization 자체가 아니라, localization 이후 navigation/planning에서 사용할 3D perception layer이다.

전체 흐름:

```text
Mapping / Localization
-> map, ground map, robot pose 확보
-> dddmr_perception_3d
-> 장애물 / 지면 / 금지구역 / 속도제한 구역을 graph 형태로 관리
-> global planner / local planner가 사용
```

핵심 역할은 세 가지이다.

```text
1. 저장된 ground point cloud를 이용해 주행 가능한 ground graph 생성
2. 실시간 LiDAR / depth camera point cloud로 장애물 marking / clearing
3. speed limit layer, no-entry layer 같은 영역 제한 반영
```

localization이 "내가 어디 있는가"를 담당한다면, `dddmr_perception_3d`는 "현재 map에서 어디를 지나갈 수 있는가"를 판단하는 layer이다.

## 7. Ground Graph와 Ground Point Cloud

ground graph를 사용하려면 지면 point cloud가 필요하다.

mapping 단계에서 생성된 결과물 중 `ground.pcd`가 중요하다.

```text
ground.pcd
-> ground node 생성
-> 인접 ground node 연결
-> 주행 가능 graph 구성
```

장애물 point cloud만으로는 충분하지 않다. 장애물 point cloud는 "못 가는 곳"을 알려주지만, ground graph는 먼저 "갈 수 있는 지면"을 정의한다.

발표용 문장:

> ground graph는 지면 point cloud를 기반으로 생성된다. mapping 단계에서 저장된 `ground.pcd`를 사용해 주행 가능한 지면 노드를 만들고, 이후 실시간 센서 데이터로 장애물 영역을 marking/clearing하여 planner가 사용할 traversability map을 구성한다.

## 8. dddmr_global_planner

`dddmr_global_planner`는 ground graph 위에서 목표 지점까지의 전역 경로를 찾는 모듈이다.

전체 흐름:

```text
ground point cloud / ground graph
-> dddmr_perception_3d에서 traversability와 obstacle cost 반영
-> dddmr_global_planner
-> A* 기반 graph search
-> nav_msgs/Path 출력
-> local planner / controller가 추종
```

일반적인 2D grid map 기반 planner와 달리, 지면 point cloud를 노드로 사용하는 graph 기반 planner이다. 따라서 경사로나 3D 지형에서도 주행 가능한 지면을 따라 경로를 생성할 수 있다.

요약:

```text
Mapping:
ground point cloud 생성

Perception 3D:
ground graph + obstacle cost 생성

Global Planner:
graph 위에서 A* 경로 탐색

Controller:
생성된 path 추종
```

## 9. Local Planner

`dddmr_local_planner`는 global planner가 만든 전역 경로를 실제 로봇이 따라갈 수 있는 짧은 주행 명령으로 바꾸는 모듈이다.

전체 흐름:

```text
Global Planner
-> global path 생성
-> Local Planner
-> 여러 후보 trajectory 생성
-> perception_3d로 충돌 / 장애물 / 지형 cost 평가
-> 가장 좋은 trajectory 선택
-> velocity command 출력
```

역할:

```text
trajectory generator:
로봇이 앞으로 몇 초 동안 갈 수 있는 후보 궤적 생성

critics:
각 후보 궤적을 점수화

perception_3d:
현재 장애물, ground graph, speed limit, no-entry 정보 제공

recovery behavior:
경로가 막히거나 모든 trajectory가 실패했을 때 대기 / 재계획 상태로 전환
```

기존 Nav2의 DWB planner와 비슷하지만, 3D navigation 기준으로 평가한다는 점이 다르다. 일반 2D planner는 footprint를 2D polygon으로 보고 충돌을 검사하지만, 이 local planner는 point cloud와 cuboid 기반으로 3D 충돌을 고려한다.

기본 동작 주기는 보통 10 Hz이다.

```text
10 Hz = 0.1초마다 한 번씩 후보 trajectory 평가 및 명령 갱신
```

## 10. Controller

여기서 controller는 모터 제어기라기보다 navigation stack의 move base / command publisher에 가깝다.

전체 흐름:

```text
Goal
-> Global Planner: 전체 경로 생성
-> Local Planner: 현재 상황에서 최적 trajectory 선택
-> Controller / P2P Move Base
-> /cmd_vel publish
-> Go2 low-level controller가 실제 구동
```

`dddmr_p2p_move_base`가 이 역할을 한다. RViz나 action으로 목표가 들어오면 action server가 목표를 받고, 내부 FSM이 상태를 관리한다.

주요 상태:

```text
planning
-> heading alignment
-> controlling
-> goal heading alignment
-> recovery / waiting
```

동작 방식:

```text
1. global planner에게 목표까지의 path 요청
2. local planner에게 현재 path를 따라갈 수 있는 최적 trajectory 요청
3. local planner가 찾은 vx, vy, angular_z를 /cmd_vel 또는 /cmd_vel_stamped로 publish
4. 경로가 막히거나 TF / perception 문제가 있으면 zero velocity publish
5. 계속 실패하면 recovery behavior로 전환
```

발표용 문장:

> Controller는 global/local planner의 결과를 바탕으로 로봇이 지금 실행해야 할 속도 명령을 생성하는 마지막 단계이며, 장애물이나 TF 오류가 있으면 안전을 위해 zero velocity를 publish하고 재계획 또는 recovery로 전환한다.

## 11. 전체 파이프라인 요약

최종적으로 현재 Go2 navigation 구조는 다음과 같이 정리할 수 있다.

```text
1. Mapping
   LiDAR point cloud를 이용해 map.pcd, ground.pcd, poses.pcd 생성

2. Localization
   현재 LiDAR feature와 저장된 map을 정합해 map -> odom TF 생성

3. Perception 3D
   ground.pcd 기반 ground graph 생성 및 실시간 장애물 cost 반영

4. Global Planner
   ground graph 위에서 목표까지의 전역 경로 생성

5. Local Planner
   현재 주변 상황을 보고 짧은 후보 trajectory 평가

6. Controller
   선택된 trajectory를 /cmd_vel로 변환해 Go2에 전달
```

한 문장 요약:

> 현재 Go2 시스템은 LeGO-LOAM 계열 mapping/localization을 기반으로 하되, 실제 센서 구성에 맞춰 `base_footprint` 기준 ground 추정을 사용하고, 생성된 ground point cloud를 perception graph와 planner가 활용해 3D navigation을 수행한다.
