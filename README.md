# ROS 2 Forklift Warehouse Workspace

Workspace nay duoc chot de demo theo mot luong chay on dinh:

`Gazebo + RViz + camera mo phong + visual odom + sim_wheel_odom EKF + Nav2`

Entry point chinh de chay va test thu cong la:

`./tools/run_visual_nav_manual.sh`

README nay chi huong dan cho luong chay do.

## Moi truong muc tieu

- `WSL Ubuntu-22.04`
- `ROS 2 Humble`
- `Gazebo Classic 11`
- `Nav2`

## Cau truc lien quan

- `src/warehouse_visual_localization`
  Pipeline visual navigation dang duoc dung.
- `src/forklift_nav_bringup`
  World, map, Nav2 stack va cac launch/phu tro can cho manual visual mode.
- `src/gazebo_nav_goal_tool`
  Nut `Set Nav Goal` trong Gazebo.
- `tools/source_visual_localization_env.sh`
  Source ROS overlay va setup WSLg/OpenGL.
- `tools/run_visual_nav_manual.sh`
  Lenh chay chinh de demo.

## Cai dat

Clone repo va submodule:

```bash
git clone --recurse-submodules git@github.com:TueNguyen2006/ros2_forklift_warehouse_ws.git
cd /home/tuenguyen/ros2_forklift_warehouse_ws
```

Neu clone truoc do chua co submodule:

```bash
git submodule update --init --recursive
```

Bootstrap phu thuoc:

```bash
./tools/bootstrap_ros2_humble.sh
```

Build workspace:

```bash
./tools/build_workspace.sh
```

Artifact build/install mac dinh nam o:

`/home/tuenguyen/ros2_forklift_warehouse_artifacts`

## Cach chay chinh

```bash
cd /home/tuenguyen/ros2_forklift_warehouse_ws
source /home/tuenguyen/ros2_forklift_warehouse_ws/tools/source_visual_localization_env.sh
./tools/run_visual_nav_manual.sh
```

Script nay se:

- dong `gzserver`, `gzclient`, `rviz2` cu neu con treo
- mo `Gazebo`
- mo `RViz`
- spawn forklift o mode `drive_model:=planar`
- chay `RGB-D visual odometry`
- fuse `visual odom` voi `/sim_wheel_odom` bang `EKF`
- dua pose cho Nav2 qua chuoi TF `map -> odom -> base_footprint`

## Pipeline pose hien tai

Pose runtime cua forklift khong doc truc tiep vi tri chinh xac tu simulator.

Pipeline dang duoc dung:

`cmd_vel -> planar_move -> than xe di chuyen trong Gazebo`

`than xe di chuyen -> /sim_wheel_odom`

`RGB-D camera -> RTAB-Map RGB-D odometry -> visual odom`

`sim_wheel_odom + visual odom -> EKF -> /odom`

`Nav2 doc TF / odom da fuse de plan va follow path`

## Cach test thu cong

Sau khi chay script:

1. Doi `Gazebo` va `RViz` len day du.
2. Trong `Gazebo`, bam `Set Nav Goal`.
3. Click xuong san de gui goal.
4. Quan sat:
   - path planner trong `RViz`
   - robot di chuyen trong `Gazebo`
   - camera RGB/depth/stereo trong `RViz`

Ban cung co the gui goal bang `Nav2 Goal` trong RViz, nhung luong test khuyen nghi hien tai la goal tu Gazebo.

## Topic va frame quan trong

Camera:

- `/rgb_camera/image_raw`
- `/depth_camera/depth/image_raw`
- `/stereo_left_camera/image_raw`
- `/stereo_right_camera/image_raw`

Odometry / TF:

- `/sim_wheel_odom`
- `/visual_odom`
- `/odometry/filtered`
- `map -> odom -> base_footprint`

Navigation:

- `/plan`
- `/cmd_vel`
- `/goal_pose`

## Ghi chu WSLg

Neu script env bao:

`/mnt/shared_memory is missing`

thi Gazebo/RViz co the bi `COPY MODE`, thumbnail hong, hoac man hinh trang. Cach xu ly:

```powershell
wsl --shutdown
```

Mo lai `Ubuntu-22.04`, source lai env va chay lai script.

## Pham vi README nay

README nay co y chi tap trung vao mode dang on dinh nhat:

- `run_visual_nav_manual.sh`
- `drive_model:=planar`
- `localization:=false`
- `use_wheel_odom_fusion:=true`

Nhung script/launch khac trong repo duoc giu lai cho nghien cuu va debug, nhung khong phai duong chay chinh trong tai lieu nay.
