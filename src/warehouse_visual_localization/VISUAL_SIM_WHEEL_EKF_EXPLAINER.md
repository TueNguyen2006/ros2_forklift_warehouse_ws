# Giải Thuật Hợp Nhất `visual_odom`, `sim_wheel_odom` Và EKF Trong Repo Này

## 1. Mục tiêu của pipeline

Mục tiêu của pipeline hiện tại là làm cho xe nâng trong mô phỏng **không phụ thuộc vào pose toàn cục chính xác của Gazebo** khi thực hiện navigation.

Thay vào đó, hệ thống sử dụng hai nhóm thông tin cảm biến:

- **Odometry nội tại**: `sim_wheel_odom`
- **Odometry theo môi trường nhìn thấy được**: `visual_odom`

Sau đó hai nguồn này được hợp nhất bằng **EKF** để tạo ra một ước lượng trạng thái ổn định hơn:

- pose trong hệ `odom`
- vận tốc tuyến tính
- vận tốc quay

Ước lượng này chính là dữ liệu mà Nav2 dùng để bám đường và điều khiển xe.

## 2. Bài toán ước lượng trạng thái

Về bản chất, bài toán ở đây là bài toán **state estimation**.

Ta cần ước lượng trạng thái robot theo thời gian:

```text
x_k = [p_x, p_y, yaw, v_x, yaw_rate]
```

Trong đó:

- `p_x, p_y`: vị trí tương đối trong hệ `odom`
- `yaw`: góc hướng
- `v_x`: vận tốc tiến
- `yaw_rate`: tốc độ quay

Ta không đo trực tiếp trạng thái này từ một nguồn tuyệt đối hoàn hảo, mà phải suy ra nó từ các cảm biến khác nhau.

## 3. Nguồn đo thứ nhất: `sim_wheel_odom`

`sim_wheel_odom` đóng vai trò giống như **encoder odometry** hoặc **wheel odometry**.

Nó trả lời câu hỏi:

- robot vừa lăn thêm bao nhiêu
- vừa quay thêm bao nhiêu
- vận tốc hiện tại là bao nhiêu

### 3.1. Ý nghĩa giải thuật

Nguồn này là một dạng **dead reckoning**:

- từ chuyển động bánh xe hoặc mô hình chuyển động
- tích lũy dần để suy ra pose hiện tại

Dead reckoning có ưu điểm:

- cập nhật nhanh
- liên tục
- ít bị mất tín hiệu đột ngột

Nhưng nhược điểm chung là:

- sai số tích lũy theo thời gian
- nếu chỉ dùng một mình thì quỹ đạo sẽ drift

### 3.2. Đặc điểm trong repo hiện tại

Trong mode hiện tại, robot chạy với `drive_model:=planar`, nên `sim_wheel_odom` là một odometry mô phỏng rất sạch:

- gần như không có trượt bánh thực
- ít nhiễu
- ít gián đoạn

Vì vậy trong bài toán hiện tại, `sim_wheel_odom` là một nguồn **rất mạnh về tính liên tục**.

## 4. Nguồn đo thứ hai: `visual_odom`

`visual_odom` là odometry sinh ra từ camera RGB-D thông qua RTAB-Map.

Nó trả lời câu hỏi:

- dựa trên các khung hình liên tiếp, robot đã dịch chuyển như thế nào trong môi trường

### 4.1. Ý nghĩa giải thuật

Nguồn này là một dạng **frame-to-frame motion estimation**:

1. lấy ảnh RGB và ảnh depth ở thời điểm `k`
2. lấy ảnh RGB và ảnh depth ở thời điểm `k+1`
3. tìm các đặc trưng hình ảnh ổn định
4. ghép cặp đặc trưng giữa hai khung
5. dùng ràng buộc hình học + thông tin chiều sâu để ước lượng phép biến đổi tương đối

Kết quả là:

```text
T_(k -> k+1)
```

tức là robot đã di chuyển tương đối từ khung trước sang khung sau như thế nào.

### 4.2. Ưu điểm

Khác với wheel odom, `visual_odom` bám vào môi trường quan sát được nên:

- có khả năng sửa drift tốt hơn
- giúp pose phản ánh quỹ đạo nhìn thấy từ cảnh thực
- đặc biệt hữu ích cho `x`, `y` và `yaw`

### 4.3. Nhược điểm

`visual_odom` dễ suy yếu khi:

- ảnh ít đặc trưng
- depth camera mất dữ liệu
- robot quay quá gắt
- motion blur lớn
- có ít inlier trong quá trình đăng ký hình học

Khi đó có thể xuất hiện lỗi kiểu:

```text
Registration failed: Not enough inliers
```

Nếu chỉ dùng `visual_odom`, pose runtime có thể đứng yên dù robot vẫn đang chuyển động trong Gazebo.

## 5. Vì sao cần hợp nhất hai nguồn này

Hai nguồn đo trên có tính chất bổ sung cho nhau:

### `sim_wheel_odom`

Mạnh ở:

- liên tục
- mượt
- luôn có vận tốc

Yếu ở:

- drift theo thời gian
- không dùng trực tiếp thông tin từ môi trường nhìn thấy

### `visual_odom`

Mạnh ở:

- sửa quỹ đạo theo môi trường
- tốt cho pose khi camera bám tốt

Yếu ở:

- dễ mất bám
- có thể gián đoạn
- không nên dùng một mình cho vòng điều khiển khi dữ liệu ảnh không ổn định

Tư tưởng hợp nhất là:

- dùng `sim_wheel_odom` làm nền chuyển động liên tục
- dùng `visual_odom` để hiệu chỉnh pose khi môi trường quan sát được đủ tốt

## 6. EKF làm gì trong bài toán này

EKF ở đây là bộ lọc trạng thái hợp nhất nhiều cảm biến để ước lượng:

```text
odom -> base_footprint
```

### 6.1. Mô hình khái niệm

EKF có hai bước lặp lại liên tục:

1. **Prediction**
   - dự đoán trạng thái mới từ trạng thái cũ theo mô hình động học

2. **Correction**
   - nhận dữ liệu đo mới
   - so sánh dự đoán với quan sát
   - hiệu chỉnh lại trạng thái

Trong bài toán này:

- prediction được hỗ trợ mạnh bởi `sim_wheel_odom`
- correction được hỗ trợ mạnh bởi `visual_odom`

### 6.2. Dạng trực giác của bài toán

Giả sử tại thời điểm `k`:

- wheel odom nói robot đã đi thêm một đoạn nhỏ
- visual odom cũng nói robot đã đi, nhưng theo hướng hơi khác một chút vì nhìn thấy môi trường

EKF không chọn cứng một trong hai.

Nó làm điều sau:

- nếu cả hai gần nhau, EKF tạo ra một trạng thái hợp nhất ổn định
- nếu visual tốt, EKF để visual kéo pose về quỹ đạo hợp lý hơn
- nếu visual yếu hoặc mất tạm thời, EKF vẫn giữ được chuyển động nhờ wheel odom

## 7. Trạng thái nào đang được fuse trong repo

Trong cấu hình hiện tại:

### Từ `sim_wheel_odom`

EKF sử dụng:

- `x`
- `y`
- `yaw`
- `v_x`
- `yaw_rate`

### Từ `visual_odom`

EKF sử dụng:

- `x`
- `y`
- `yaw`

Điều này rất quan trọng về mặt giải thuật:

- `sim_wheel_odom` không chỉ đóng góp pose, mà còn đóng góp **thông tin vận tốc**
- `visual_odom` chủ yếu đóng vai trò **hiệu chỉnh pose**

Do đó, nếu camera hụt trong thời gian ngắn:

- hệ vẫn còn vận tốc liên tục
- `/odom` không bị đóng băng ngay

## 8. Vì sao EKF giúp pose ổn định hơn

Nếu chỉ dùng `visual_odom`:

- pose có thể rất chính xác trong lúc bám tốt
- nhưng dễ bị dừng cập nhật khi camera mất bám

Nếu chỉ dùng `sim_wheel_odom`:

- pose mượt và đều
- nhưng lâu dài sẽ drift

Khi dùng EKF:

- pose ngắn hạn không bị gãy vì wheel odom đỡ bên dưới
- pose dài hơn không drift nhanh như wheel-only vì visual đang kéo lại

Nói ngắn:

- wheel odom tạo **độ ổn định**
- visual odom tạo **độ đúng hướng theo môi trường**

## 9. Vì sao EKF giúp velocity ổn định hơn

Nav2 local controller không chỉ cần biết robot đang ở đâu, mà còn cần biết:

- robot có đang thực sự di chuyển không
- tốc độ hiện tại là bao nhiêu
- hướng quay hiện tại là bao nhiêu

Nếu vận tốc ước lượng bị đứt hoặc pose update không đều:

- controller sẽ khó dự đoán quỹ đạo tiếp theo
- progress checker dễ kết luận sai rằng robot không tiến triển
- lệnh điều khiển sẽ dao động hoặc bị ngắt

Trong pipeline này:

- `sim_wheel_odom` cung cấp chuyển động mượt
- EKF biến nó thành `/odom` ổn định cho Nav2

Vì vậy bộ điều khiển có phản hồi tốt hơn và quỹ đạo theo dõi ổn định hơn.

## 10. Vì sao điều này giúp bám path tốt hơn

Khả năng bám path của robot phụ thuộc mạnh vào chất lượng của `/odom`.

Nếu `/odom` tốt:

- controller biết robot đang lệch path bao nhiêu
- biết đang tiến hay đứng
- biết tốc độ quay hiện tại có phù hợp không

Nếu `/odom` xấu:

- robot có thể quay quá tay
- dừng bất thường
- planner vẫn có path nhưng follower không bám được

EKF giúp cải thiện path tracking vì nó làm cho estimate trạng thái:

- liên tục hơn
- ít bị freeze hơn
- mượt hơn cho bộ điều khiển
- đáng tin cậy hơn cho costmap cục bộ trong frame `odom`

## 11. Nguồn nào ảnh hưởng nhiều hơn

Không thể diễn đạt đúng bằng kiểu:

- 60% wheel
- 40% visual

Vì EKF không hoạt động theo tỉ lệ cố định như vậy.

Ảnh hưởng phụ thuộc vào:

- covariance của từng nguồn
- biến trạng thái nào đang được fuse
- chất lượng đo ở thời điểm hiện tại
- ngưỡng reject outlier

Tuy nhiên, có thể hiểu đúng theo vai trò:

### `sim_wheel_odom` ảnh hưởng mạnh hơn ở:

- độ liên tục của chuyển động
- vận tốc
- việc tránh cho `/odom` bị đóng băng

### `visual_odom` ảnh hưởng mạnh hơn ở:

- hiệu chỉnh pose
- sửa sai lệch quỹ đạo
- giữ cho `x`, `y`, `yaw` bám môi trường hơn

Vì vậy trong hệ hiện tại:

- wheel odom là “xương sống chuyển động”
- visual odom là “cơ chế sửa pose theo cảnh quan sát được”

## 12. Liên hệ với `localization:=false`

Trong mode hiện tại thường chạy với `localization:=false`.

Điều đó có nghĩa là:

- hệ **vẫn dùng cảm biến** để ước lượng chuyển động
- nhưng **chưa chạy bước định vị toàn cục thật sự theo map**

Vì thế:

- `odom -> base_footprint` là sensor-based
- nhưng `map -> odom` trong mode này chưa phải visual global localization hoàn chỉnh

Do đó, tài liệu này tập trung vào phần đang hoạt động rất tốt hiện nay:

- ước lượng chuyển động cục bộ
- hợp nhất odometry
- cải thiện bám path

## 13. Kết luận

Về mặt giải thuật, pipeline hiện tại thành công vì nó không đặt toàn bộ gánh nặng lên một cảm biến duy nhất.

Thay vào đó:

- `sim_wheel_odom` cung cấp chuyển động liên tục, mượt và ổn định
- `visual_odom` cung cấp khả năng hiệu chỉnh pose dựa trên môi trường nhìn thấy được
- EKF hợp nhất hai nguồn này để tạo `/odom` đủ ổn định cho Nav2

Chính cơ chế đó làm cho:

- pose ít bị đứng yên đột ngột hơn
- velocity đáng tin cậy hơn
- controller bám path ổn định hơn
- toàn bộ visual navigation trong repo hoạt động tốt hơn rõ rệt so với khi chỉ dùng visual odometry đơn lẻ
