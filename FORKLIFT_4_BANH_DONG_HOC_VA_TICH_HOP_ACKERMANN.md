# Research: Xe Nâng 4 Bánh, Động Học, Động Lực Học, Và Cách Tích Hợp Vào Pipeline Hiện Tại

## 1. Mục tiêu tài liệu

Tài liệu này trả lời 4 câu hỏi:

1. Xe nâng 4 bánh kiểu counterbalance ngoài thực tế thường bố trí bánh như thế nào?
2. Mô hình động học và động lực học nào là phù hợp nhất để mô tả nó?
3. Mô hình đó khác gì so với pipeline `Ackermann` đang có trong repo?
4. Muốn giữ độ ổn định như mode đang chạy tốt hiện tại nhưng tiến dần sang mô phỏng thật hơn thì nên đi theo lộ trình nào?

Tài liệu này thiên về giải thuật và mô hình hóa, không tập trung vào mô tả code.

## 2. Kết luận ngắn

Kết luận quan trọng nhất là:

- Với đa số xe nâng 4 bánh kiểu counterbalance, cấu hình phổ biến là `front-wheel drive + rear-wheel steering`, không phải `rear-wheel drive + rear-wheel steering`.
- Vì cầu sau là cầu lái và thường có trục quay/pivot, xe nâng 4 bánh không hành xử giống ô tô Ackermann tiêu chuẩn.
- Nếu chỉ nhìn ở mức động học phẳng tốc độ thấp, xe nâng 4 bánh có thể quy đổi về một mô hình "single-track rear-steer".
- Nếu muốn mô phỏng thật hơn, phần khó không nằm ở planner toàn cục mà nằm ở `contact tire-ground`, `rear axle pivot`, `load transfer`, `yaw-rate stability`, và cách biến lệnh `cmd_vel` thành lệnh truyền động/cầu lái đúng vật lý.
- Pipeline hiện tại trong repo về mặt tư duy tổng thể là đúng hướng: dùng cảm biến để suy ra pose, rồi để Nav2 dùng pose đó. Vấn đề lớn còn lại nằm ở nhánh `realistic`: mô hình thân xe, drive-train, steering, và contact chưa đủ "xe nâng thật".

## 3. Xe nâng 4 bánh thực tế vận hành như thế nào

### 3.1. Kiến trúc cơ khí điển hình

Với xe nâng counterbalance 4 bánh:

- hai bánh trước thường là bánh chịu tải chính
- cầu trước thường là cầu kéo hoặc cầu nhận lực kéo
- hai bánh sau là cầu lái
- cầu sau thường có khả năng quay quanh một điểm pivot ở giữa trục

Điểm rất quan trọng là dù xe có 4 bánh, về mặt ổn định tĩnh nó thường được xem như hệ `3-point support`:

- tiếp điểm bánh trước trái
- tiếp điểm bánh trước phải
- tâm pivot của cầu lái phía sau

Điều này giải thích vì sao xe nâng không "ổn định kiểu ô tô 4 bánh thường", và cũng giải thích vì sao khi mô phỏng, chỉ cần sai ở `CG`, `rear axle pivot`, hoặc `contact stiffness` là xe có thể nghiêng, trượt, hoặc lùi/trôi bất thường.

### 3.2. Bằng chứng từ tài liệu nguồn

Nguồn thực hành công nghiệp và manual cho thấy rõ:

- tài liệu CLARK nhắc tới `rear steer wheels` và tách riêng `drive` với `steer` trong bảo trì và điều khiển
- tài liệu Taylor nêu rõ xe được đỡ bởi `drive axle tires` ở phía trước và `steer axle pivot` ở phía sau, tạo thành `stability triangle`

Điều này khớp với nhận định rằng repo hiện tại không sai ở ý tưởng "cầu trước kéo, cầu sau lái". Sai số hiện tại chủ yếu nằm ở mô hình động lực học và chấp hành.

## 4. Mô hình động học phẳng phù hợp nhất ở tốc độ thấp

## 4.1. Vì sao không dùng nguyên xi mô hình ô tô

Mô hình ô tô Ackermann chuẩn thường giả sử:

- bánh trước lái
- bánh sau tạo vận tốc dọc chính
- gốc thân xe đặt gần trung điểm cầu sau

Trong khi đó xe nâng 4 bánh counterbalance thường:

- cầu sau lái
- cầu trước kéo
- tâm quay tức thời nằm gần trục trước hơn khi quay gắt

Do đó, nếu vẫn dùng cấu trúc `Ackermann` như một interface điều khiển, phải đổi biến và đổi quy ước dấu, không thể bê nguyên mô hình ô tô sang.

## 4.2. Mô hình single-track rear-steer

Ở vận tốc thấp, bỏ qua trượt ngang lớn, ta có thể dùng mô hình bicycle tương đương cho xe nâng cầu sau lái.

Ký hiệu:

- `x, y`: vị trí thân xe trong mặt phẳng
- `psi`: yaw của thân xe
- `v`: vận tốc dọc của thân xe
- `L`: khoảng cách trục trước - trục sau
- `delta_r`: góc lái tương đương của cầu sau

Nếu chọn điểm tham chiếu tại trung điểm cầu trước, một dạng mô hình thuận tiện là:

```math
\dot{x} = v \cos(\psi)
```

```math
\dot{y} = v \sin(\psi)
```

```math
\dot{\psi} = - \frac{v}{L}\tan(\delta_r)
```

Dấu âm xuất hiện vì đây là `rear steer`, không phải `front steer`.

Ý nghĩa rất thực dụng:

- cùng một độ cong quỹ đạo `kappa`
- xe nâng cầu sau lái cần góc lái tương đương ngược dấu với xe Ackermann cầu trước lái

Nói ngắn gọn:

```math
\delta_{ack} \approx -\delta_r
```

và

```math
\kappa = \frac{\tan(\delta_{ack})}{L} = -\frac{\tan(\delta_r)}{L}
```

Từ đó, nếu controller phía trên xuất `v, omega`, ta có thể đổi sang cầu sau lái bằng:

```math
\delta_r = - \arctan\left(\frac{L\omega}{v}\right)
```

khi `v != 0`.

Đây là công thức quan trọng nhất nếu muốn giữ nguyên pipeline Nav2 ở tầng trên nhưng thay lớp chấp hành sang xe nâng thật hơn.

## 4.3. Góc bánh trong và ngoài của cầu sau

Nếu cầu sau có hai bánh lái riêng trái/phải và muốn thỏa điều kiện lăn không trượt theo Ackermann hình học, thì không nên cho hai bánh sau cùng một góc tuyệt đối.

Ký hiệu:

- `T_r`: bề rộng cầu sau
- `R`: bán kính quay của quỹ đạo tham chiếu

Khi quay, độ lớn góc lái trong/ngoài của bánh sau là:

```math
\tan(\delta_{r,in}) = \frac{L}{R - T_r/2}
```

```math
\tan(\delta_{r,out}) = \frac{L}{R + T_r/2}
```

và góc lái tương đương của cầu sau:

```math
\tan(\delta_{r,eq}) = \frac{L}{R} = L\kappa
```

Trong triển khai thực tế:

- dấu `+/-` của từng bánh phụ thuộc quy ước quay trái/phải
- điều quan trọng là bánh trong phải quay lớn hơn bánh ngoài theo độ lớn

Nếu bỏ qua sai khác này và ép hai bánh sau cùng góc, mô phỏng có thể vẫn chạy ở tốc độ thấp, nhưng khi quay gắt hoặc ma sát lớn thì lực ngang và moment sai sẽ tăng đáng kể.

## 5. Ma trận động học tương đương với Ackermann hiện tại

Tài liệu roboticsbook cho mô hình ô tô Ackermann chuẩn:

```math
\begin{bmatrix}
\dot{x}\\
\dot{y}\\
\dot{\psi}\\
\dot{\phi}
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi\\
\sin\psi\\
\frac{1}{L}\tan\phi\\
0
\end{bmatrix}u_1
+
\begin{bmatrix}
0\\
0\\
0\\
1
\end{bmatrix}u_2
```

với:

- `u_1 = v`
- `u_2 = dot(phi)`

Với xe nâng cầu sau lái, chỉ cần đổi `phi -> -delta_r` ở mức động học phẳng là ta có thể tái dùng gần như toàn bộ logic độ cong, yaw-rate, và tracking của controller tầng cao.

Nói cách khác:

- planner không cần biết xe là front-steer hay rear-steer
- local controller hoặc adapter phải biết
- phần adapter chịu trách nhiệm đổi `cmd_vel` thành `rear-steer reference`

## 6. Mô hình động lực học phẳng tối thiểu

Khi cần mô phỏng thật hơn, động học không đủ nữa. Lúc này phải nhìn vào lực ngang lốp và yaw dynamics.

Ký hiệu:

- `u`: vận tốc dọc danh định
- `v_y`: vận tốc ngang thân xe
- `r = dot(psi)`: yaw rate
- `m`: khối lượng
- `I_z`: moment quán tính quanh trục đứng
- `l_f, l_r`: khoảng cách từ CG tới cầu trước/sau
- `C_f, C_r`: độ cứng góc trượt của lốp trước/sau

Phương trình phẳng chuẩn:

```math
m(\dot{v}_y + ur) = F_{yf} + F_{yr}
```

```math
I_z\dot{r} = l_f F_{yf} - l_r F_{yr}
```

Với xe nâng rear-steer:

- cầu trước không lái, nên góc trượt trước xấp xỉ

```math
\alpha_f \approx -\frac{v_y + l_f r}{u}
```

- cầu sau có điều khiển lái, nên

```math
\alpha_r \approx \delta_r - \frac{v_y - l_r r}{u}
```

và lực ngang tuyến tính:

```math
F_{yf} = C_f \alpha_f
```

```math
F_{yr} = C_r \alpha_r
```

Khi thay vào, ta thu được mô hình trạng thái tối thiểu:

```math
\dot{\mathbf{x}} = A\mathbf{x} + B\delta_r
```

với:

```math
\mathbf{x} = \begin{bmatrix} v_y \\ r \end{bmatrix}
```

Một dạng ma trận hóa phổ biến là:

```math
A =
\begin{bmatrix}
-\frac{C_f + C_r}{mu} & -u - \frac{C_f l_f - C_r l_r}{mu} \\
-\frac{C_f l_f - C_r l_r}{I_z u} & -\frac{C_f l_f^2 + C_r l_r^2}{I_z u}
\end{bmatrix}
```

```math
B =
\begin{bmatrix}
\frac{C_r}{m} \\
-\frac{C_r l_r}{I_z}
\end{bmatrix}
```

Lưu ý:

- dấu cụ thể của từng phần tử phụ thuộc hệ trục và quy ước góc
- nhưng cấu trúc vật lý không đổi: ở xe nâng rear-steer, ngõ vào điều khiển chính đi vào qua cầu sau, không phải cầu trước

Ý nghĩa thực tế:

- nếu local controller chỉ bám hình học mà không khống chế `yaw-rate`, `lateral acceleration`, và `steering rate`, xe rất dễ quay quá nhanh hoặc tự mất bám khi gặp cua gắt
- đây chính là kiểu triệu chứng đã thấy ở nhánh `realistic`

## 7. Động lực học lật và vì sao xe nâng khó mô phỏng hơn ô tô nhỏ

## 7.1. Stability triangle

Xe nâng counterbalance có đặc trưng rất khác ô tô:

- nó không được đỡ "cứng" trên 4 góc
- cầu sau thường có pivot
- tải trọng trên càng nâng làm CG dịch mạnh theo cả trục dọc lẫn trục đứng

Do đó, vùng ổn định đầu tiên cần nghĩ tới không phải hình chữ nhật 4 bánh mà là tam giác ổn định.

Theo manual và literature:

- hai điểm trước là bánh cầu kéo
- điểm thứ ba là tâm pivot cầu lái phía sau

Nếu hình chiếu trọng tâm hợp lực vượt ra khỏi tam giác này, xe bắt đầu có nguy cơ lật.

## 7.2. Mô hình roll tối thiểu

Một mô hình roll tối thiểu có thể viết:

```math
I_x \ddot{\phi} + c_\phi \dot{\phi} + k_\phi \phi = m_s h_s a_y + M_{load}
```

trong đó:

- `phi`: góc nghiêng thân xe
- `I_x`: moment quán tính roll
- `c_phi`: damping roll
- `k_phi`: độ cứng roll tương đương
- `m_s`: khối lượng phần thân chịu lật
- `h_s`: độ cao CG hiệu dụng
- `a_y`: gia tốc ngang

với:

```math
a_y \approx u r + \dot{v}_y
```

Một chỉ số rất hay dùng là `LTR`:

```math
LTR = \frac{F_{z,left} - F_{z,right}}{F_{z,left} + F_{z,right}}
```

hoặc ở mức xấp xỉ:

```math
LTR \propto \frac{2 h_{CG}}{g t} a_y
```

với `t` là track width.

Ý nghĩa:

- `|LTR|` càng gần 1 thì càng gần mất tải ở một bên
- khi nâng hàng cao, `h_CG` tăng, cùng một góc cua sẽ nguy hiểm hơn rất nhiều

## 7.3. Hệ quả cho điều khiển

Muốn xe nâng "thật" thì không thể chỉ điều khiển bằng:

- bám heading
- bám cross-track error

Mà phải ràng buộc thêm:

- yaw-rate
- steering rate
- lateral acceleration
- speed according to curvature
- speed according to load height

Nói cách khác, controller tốt cho xe nâng phải có ít nhất một lớp `speed-curvature scheduling`.

## 8. Đối chiếu trực tiếp với repo hiện tại

## 8.1. Repo hiện tại đang mô hình hóa gì

Trong file [rear_steer_forklift.urdf.xacro](/home/tuenguyen/ros2_forklift_warehouse_ws/src/forklift_description_realistic/urdf/rear_steer_forklift.urdf.xacro:374):

- `front_left_wheel_joint` và `front_right_wheel_joint` nhận `command_interface velocity`
- `rear_left_steering_joint` và `rear_right_steering_joint` nhận `command_interface position`

Điều đó có nghĩa:

- cầu trước đang đóng vai trò traction
- cầu sau đang đóng vai trò steering

Tức là đúng với cấu hình xe nâng 4 bánh counterbalance điển hình.

Trong file [rear_steer_controller_visual.yaml](/home/tuenguyen/ros2_forklift_warehouse_ws/src/warehouse_visual_localization/config/rear_steer_controller_visual.yaml:15):

- controller đang là `ackermann_steering_controller/AckermannSteeringController`
- `front_steering: false`
- `rear_wheels_names` lại map vào các joint lái sau
- `front_wheels_names` map vào các joint kéo trước

Nghĩa là repo đang dùng controller Ackermann như một abstraction cho `rear-steer vehicle`.

Trong file [visual_pose.launch.py](/home/tuenguyen/ros2_forklift_warehouse_ws/src/warehouse_visual_localization/launch/visual_pose.launch.py:44):

- khi `drive_model:=rear_steer`, lệnh điều khiển được đẩy tới `/rear_steer_controller/reference`
- EKF sẽ đọc odom từ `/rear_steer_controller/odometry`
- còn mode đang ổn định hơn hiện tại là `drive_model:=planar`

Điều này cho thấy vấn đề hiện nay không nằm ở Nav2 hay visual odom tầng cao. Vấn đề nằm ở lớp chấp hành và mô hình động lực học của `rear_steer`.

## 8.2. Vì sao nhánh realistic còn bất ổn

Từ góc nhìn mô hình hóa, có ít nhất 5 nguyên nhân lớn:

1. `AckermannSteeringController` không phải là mô hình động cơ - vi sai - lốp - mặt sàn của xe nâng.
2. Cầu sau của xe nâng thật có pivot, compliance, saturation, giới hạn tốc độ lái và response phi tuyến rõ rệt.
3. Lốp xe nâng thường là solid/cushion tire, không giống lốp ô tô thông thường.
4. Xe nâng nhạy với vị trí CG, chiều cao hàng, và load transfer hơn xe car-like rất nhiều.
5. Nếu contact model quá cứng hoặc PID truyền động không hợp lý, Gazebo rất dễ sinh ra:
   - tự trôi
   - rung
   - quay tại chỗ
   - thân xe nghiêng/lún sai
   - odom và mô hình hiển thị lệch nhau

## 9. Tích hợp vào pipeline Ackermann hiện tại như thế nào để vẫn ổn định như cũ

## 9.1. Cái nên giữ nguyên

Những phần sau không nhất thiết phải viết lại:

- map có sẵn cho RViz/Nav2
- global planner
- path follower tầng cao nếu đầu ra vẫn là `v, omega`
- visual odom
- EKF fusion

Đây là điểm rất quan trọng. Bài toán của bạn không phải "đập hết làm lại", mà là đổi lớp phương tiện từ `planar surrogate` sang `rear-steer forklift dynamics`.

## 9.2. Adapter đúng về mặt hình học

Nếu local planner hoặc Nav2 controller vẫn xuất:

- `v`
- `omega`

thì lớp adapter nên đổi sang:

- `rear steer angle`
- `wheel traction command`

với:

```math
\delta_r = -\arctan\left(\frac{L\omega}{v}\right)
```

và:

```math
v_{wheel} = \frac{v}{r_w}
```

nếu dùng tốc độ góc bánh quy đổi qua bán kính `r_w`.

Như vậy:

- planner không cần biết xe là rear-steer
- robot base vẫn theo quỹ đạo cũ
- nhưng phần chấp hành vật lý được đổi đúng bản chất hơn

## 9.3. Cái cần thêm để ổn định

Muốn chạy ổn định như mode `planar` đang tốt, nhánh `realistic` phải có tối thiểu:

1. `steering rate limit`
2. `steering angle limit`
3. `speed limit as function of curvature`
4. `yaw-rate damping`
5. `wheel odom + visual odom + IMU fusion`

Trong đó:

- wheel odom cho short-term velocity rất ổn
- visual odom giúp sửa drift hình học lâu dài
- IMU giúp giữ yaw-rate và roll/pitch tốt hơn khi mô phỏng vật lý thật

Nếu chỉ có wheel + visual mà không có IMU, nhánh realistic vẫn có thể chạy, nhưng yaw transient và roll disturbance sẽ khó xử lý hơn nhiều.

## 9.4. Lộ trình tích hợp khuyến nghị

Lộ trình an toàn nhất không phải là nhảy thẳng từ `planar` sang "xe nâng vật lý hoàn chỉnh", mà là:

### Bước 1

Giữ nguyên:

- Nav2
- visual odom
- EKF

nhưng thay lớp adapter `cmd_vel -> rear_steer reference` cho đúng hình học rear-steer.

### Bước 2

Thêm:

- steering rate limit
- curvature-based speed scheduling
- yaw-rate damping

để xe không quay gắt và không bị văng khi follow path cong mạnh.

### Bước 3

Thêm mô phỏng vật lý thật hơn:

- rear axle pivot đúng
- tire/contact model tốt hơn
- CG/load placement đúng
- IMU vào EKF

### Bước 4

Nếu cần độ thật cao hơn nữa, chuyển từ controller hình học sang controller động lực học:

- rear-steer MPC
- LQR trên mô hình lateral-yaw
- hoặc MPPI với motion model rear-steer và cost có ràng buộc `yaw-rate`, `a_y`, `LTR`

## 10. Trả lời trực tiếp các hiểu nhầm hay gặp

### 10.1. "Xe nâng 4 bánh có phải 2 bánh sau là bánh dẫn động không?"

Với nhiều xe nâng counterbalance 4 bánh phổ biến, câu trả lời thường là `không`.

Cấu hình phổ biến hơn là:

- `2 bánh trước dẫn động`
- `2 bánh sau lái`

Nếu bạn đang nghĩ tới một cấu hình khác, có thể đó là:

- một dòng xe chuyên dụng khác
- hoặc xe nâng điện/AGV đặc biệt có drive module khác chuẩn counterbalance truyền thống

### 10.2. "Pipeline hiện tại dành cho Ackermann có bỏ đi được không?"

Không cần bỏ.

Nếu nhìn ở tầng trên:

- planner
- path representation
- `cmd_vel`

thì vẫn tái dùng được.

Cái phải đổi là:

- motion model local
- adapter lệnh
- giới hạn động học
- mô hình vật lý ở Gazebo

### 10.3. "Tại sao mode planar đang ổn mà realistic lại khó?"

Vì `planar` đã bỏ gần hết các bậc tự do và nhiễu vật lý khó:

- không có contact lốp thật
- không có roll thật
- không có truyền động thật
- không có rear axle pivot đúng mức vật lý

Nó giống một "truth-preserving kinematic surrogate".

Còn `realistic` thì bắt đầu chạm tới:

- lực tiếp xúc
- moment quán tính
- saturation
- sai mô hình lốp
- sai mô hình steering

nên controller nào trước đây đủ tốt cho `planar` chưa chắc đủ cho `realistic`.

## 11. Kết luận dành riêng cho repo này

Nếu mục tiêu là:

`map cố định + camera/odom để định vị + xe tự chạy ổn định trong kho`

thì pipeline hiện tại đúng hướng khi:

- pose không lấy trực tiếp từ ground truth
- dùng `sim_wheel_odom + visual odom -> EKF`
- Nav2 chỉ tiêu thụ pose đã fuse

Nếu mục tiêu tiếp theo là:

`xe nâng thật hơn về động lực học`

thì việc cần làm không phải là thay planner trước, mà là hoàn thiện 3 lớp sau:

1. `rear-steer kinematic adapter`
2. `dynamic stabilization layer`
3. `Gazebo physical model` đúng với xe nâng counterbalance

Đó mới là đường đi giúp giữ được độ ổn định đang có của mode tốt hiện tại, nhưng tiến dần sang một mô phỏng "xe nâng thật" thay vì một body hình xe nâng gắn trên nền điều khiển car-like.

## 12. Nguồn tham khảo

### Nguồn kỹ thuật/giáo trình

- Dellaert, F. and Hutchinson, S., "Kinematics for Driving", *Introduction to Robotics and Perception*.
  Link: [https://www.roboticsbook.org/S62_driving_actions.html](https://www.roboticsbook.org/S62_driving_actions.html)

### Nguồn manual / công nghiệp

- CLARK Material Handling Company, operator manual `OM-747`.
  Link: [https://www.clarkmhc.com/wp-content/uploads/2024/02/OM-747.pdf](https://www.clarkmhc.com/wp-content/uploads/2024/02/OM-747.pdf)

- Taylor Machine Works, safety manual.
  Link: [https://taylorforklifts.com/assets/brochures/safety/safety-manual.pdf](https://taylorforklifts.com/assets/brochures/safety/safety-manual.pdf)

### Nguồn học thuật về động lực học xe nâng

- Pinelli, M., Giovannucci, M., Martini, A., "Analysis of the Dynamic Behavior of a Counterbalance Forklift Truck through Multibody Modelling and Simulation", ECCOMAS Multibody Dynamics 2021.
  Link: [https://cris.unibo.it/retrieve/e1dcb339-0ee9-7715-e053-1705fe0a6cc9/Pinelli_MBD2021_FP170.pdf](https://cris.unibo.it/retrieve/e1dcb339-0ee9-7715-e053-1705fe0a6cc9/Pinelli_MBD2021_FP170.pdf)

- Pinelli, M. et al., "Virtual Testing of Counterbalance Forklift Trucks: Implementation and Experimental Validation of a Numerical Multibody Model", *Machines*, 2020.
  Link: [https://www.mdpi.com/2075-1702/8/2/26](https://www.mdpi.com/2075-1702/8/2/26)

- Zhang, Y. et al., "Roll dynamic model and steering stability analysis of the counterbalance forklift truck with considering hierarchical rollover", 2024.
  Metadata page: [https://trid.trb.org/View/2387631](https://trid.trb.org/View/2387631)

## 13. Ghi chú về độ chắc chắn của kết luận

- Kết luận `front-drive + rear-steer` là kết luận mạnh, được hỗ trợ bởi manual công nghiệp và khớp với URDF hiện tại.
- Các phương trình động học ở trên là mô hình hóa chuẩn cho vận tốc thấp, ít trượt.
- Các ma trận động lực học là dạng tuyến tính hóa tối thiểu để phân tích và thiết kế controller, không phải full multibody model của xe nâng thật.
- Nếu bạn muốn mô phỏng hàng thật trên càng, mast nâng lên/xuống, hoặc giới hạn lật thật, lúc đó phải đi tiếp sang `multibody + load shift + roll + contact`.
