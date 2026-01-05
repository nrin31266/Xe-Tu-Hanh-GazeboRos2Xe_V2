# Xe-Tu-Hanh-GazeboRos2Xe_V2

Dự án mô phỏng xe tự hành trên Gazebo (ROS 2 Humble) với bản đồ đường thẳng, tránh vật cản bằng LaserScan và dừng khi tới vạch đích.

## Cấu trúc dự án (ngắn gọn)

```
src/
  my_car_description/
    models/my_car/model.sdf        # mô hình xe + lidar + diff_drive
  my_world/
    worlds/highway.world           # bản đồ đường thẳng + vật cản
  my_robot_controller/
    launch/autopilot.launch.py     # chạy Gazebo + spawn xe + điều khiển
    my_robot_controller/
      avoid_obstacle.py            # né vật cản
      finish_detector.py           # phát hiện đích
run_project.sh                     # chạy nhanh launch
clean_build.sh                     # dọn + build
check_dependencies.sh              # kiểm tra dependency
```

## Cấu trúc xe (model.sdf)

- `base_link`: khối chính 0.90 x 0.45 x 0.22, khối lượng 10 kg, có cabin và phần thân lidar giả để hiển thị.
- Lidar (`ray` sensor): đặt tại `(0.30, 0, 0.22)`, 120 mẫu, góc quét ±60°, tầm 0.12–8.0 m, publish `/gazebo_ros_laser/out`.
- 4 bánh: front_left/right, rear_left/right; bán kính 0.08 m, khoảng cách 2 bánh 0.52 m.
- `diff_drive` plugin: dùng 2 bánh sau để kéo, subscribe `/cmd_vel`, publish `/odom`, `base_link` là base frame.

## Cấu trúc map (highway.world)

- Đường thẳng dài 70 m, rộng 12 m; nền cỏ + mặt đường + vạch biên + vạch chia làn.
- Vạch xuất phát và vạch đích dạng ô cờ (checker).
- Nhiều vật cản (box/cylinder) đặt dọc đường ở các tọa độ khác nhau.
- Camera mặc định nhìn từ trên xuống toàn tuyến.

## Cách né vật cản (avoid_obstacle.py)

- Dùng LaserScan chia 3 sector: trước (-10°..10°), trái (10°..40°), phải (-40°..-10°).
- Trạng thái điều khiển:
  - `FORWARD`: chạy thẳng theo yaw ban đầu, giữ làn.
  - `TURN_OUT`: quay 90° sang trái/phải để né.
  - `PASS`: chạy song song để vượt vật cản theo khoảng cách mục tiêu.
  - `TURN_BACK`: quay lại làn ban đầu.
- Chọn hướng né theo khoảng trống trái/phải và giới hạn biên làn (y).
- `finish_detector.py` phát hiện đến đích khi `x >= 70` và publish `/race_finished`; xe sẽ phanh ngược rồi giữ dừng.

## Lệnh chạy

```
# kiểm tra dependency
./check_dependencies.sh

# dọn + build
./clean_build.sh

# chạy autopilot (Gazebo + xe + tránh vật cản)
./run_project.sh

# chạy launch cụ thể
./run_project.sh autopilot.launch.py
```

## Lệnh ROS 2 để xem topic

```
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /gazebo_ros_laser/out
ros2 topic echo /race_finished
ros2 topic hz /gazebo_ros_laser/out
ros2 topic info /cmd_vel
ros2 node list
```
