# warehouse_visual_localization

Package nay dang duoc dung theo mot mode chinh duy nhat:

`./tools/run_visual_nav_manual.sh`

Muc tieu cua mode nay la thay pose ground-truth bang pose suy ra tu cam bien mo phong, nhung van giu duoc Nav2 chay on dinh cho bai toan warehouse navigation.

## Luong chay dang duoc dung

`RGB-D camera -> RTAB-Map RGB-D odometry -> visual odom`

`Gazebo planar motion -> /sim_wheel_odom`

`visual odom + sim_wheel_odom -> EKF -> /odom`

`Nav2 dung /odom va TF da fuse de plan / follow path`

## Launch duoc dung boi script chinh

`run_visual_nav_manual.sh` goi:

```bash
ros2 launch warehouse_visual_localization nav_with_estimated_pose.launch.py \
  gui:=true \
  rviz:=true \
  headless:=false \
  localization:=false \
  use_wheel_odom_fusion:=true \
  drive_model:=planar \
  use_stability_guard:=false \
  use_collision_monitor:=false
```

Y nghia cac tham so dang duoc chot:

- `drive_model:=planar`
  Su dung `planar_move` de than xe di chuyen trong Gazebo, sau do sinh ra `/sim_wheel_odom`.
- `localization:=false`
  Khong bat `map -> odom` bang RTAB-Map localization database mode.
- `use_wheel_odom_fusion:=true`
  Bat EKF de fuse wheel odom mo phong va visual odom.

## Topic chinh

Camera:

- `/rgb_camera/image_raw`
- `/depth_camera/depth/image_raw`
- `/stereo_left_camera/image_raw`
- `/stereo_right_camera/image_raw`

Odometry:

- `/sim_wheel_odom`
- `/visual_odom`
- `/odometry/filtered`

Navigation / goal:

- `/goal_pose`
- `/plan`
- `/cmd_vel`

## Vai tro EKF trong mode nay

`visual odom` cho huong va chuyen dong tu camera, nhung de bi giat hoac mat tracking luc texture xau.

`sim_wheel_odom` rat muot va on dinh trong simulator, nhung ve ban chat van la odom tich luy.

EKF gop hai nguon nay de:

- giu van toc muot hon
- giu heading on dinh hon khi bam path
- giam truong hop visual odom dung hinh tam thoi

## Cach chay package theo duong chinh

Tu root workspace:

```bash
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /home/tuenguyen/ros2_forklift_warehouse_ws/tools/source_visual_localization_env.sh
./tools/run_visual_nav_manual.sh
```

README nay co y khong mo rong sang cac mode debug / benchmark / realistic khac. Package van con cac launch va script phu, nhung tai lieu chinh cua nhanh hien tai chi xem `run_visual_nav_manual.sh` la entry point.
