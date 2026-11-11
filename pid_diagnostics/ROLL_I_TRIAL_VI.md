# Bản thử nhánh I cho roll

Tạo model `Stabilizer_QDrone_048378_RollI.mdl` từ AutoCal đã lưu trên đĩa.
Đây là bản thử giới hạn nhỏ để kiểm tra hướng hiệu chỉnh, không phải bộ gain
autotune đã được xác nhận cho bay tự do. Không ghi đè AutoCal hoặc Stabilizer gốc.

## Tạo, kiểm tra và build cục bộ

```matlab
cd('D:\QDrone\QDrone_Control');
addpath('pid_diagnostics');
clear setup_roll_integral_trial qdrone_roll_i_step;
setup_roll_integral_trial(true);
```

Đóng model RollI trước nếu đang mở. Script chạy test thuật toán, tạo model,
Update Diagram, kiểm tra đường mô-men và giữ nguyên gain P/D, rồi build.
Không download, Start hoặc Arm.

Các đầu vào vector của hai MATLAB Function mới được nối qua Reshape riêng
để chuyển 1-D sang ma trận cột. Đầu ra mô-men I chuyển lại 1-D trước khi cộng
vào mô-men PD. Không thay kích thước tín hiệu của các gain/limiter gốc.

## Thay đổi điều khiển

- Nhánh I chỉ thêm vào mô-men roll, trước giới hạn mô-men chung.
- Tích phân sai số **tốc độ góc** từ `Sum2`, không tích phân trực tiếp góc.
- Ki roll = `0.02 Nm/rad`; Ki pitch và yaw = 0.
- Mô-men I bị giới hạn ±`0.03 Nm`, tốc độ thay đổi tối đa `0.01 Nm/s`.
- Dừng tích lũy khi lệnh motor ngoài 0..1 hoặc giới hạn mô-men đang cắt.
  Cờ giới hạn lấy từ mẫu trước qua Unit Delay 1 ms để tránh vòng đại số.
- Xóa tích phân khi Disarm, AutoCal chưa Ready, throttle <4 N, mode không
  phải Angle, góc roll/pitch vượt 10 độ, tốc độ roll vượt 30 deg/s hoặc
  tín hiệu đầu vào không hữu hạn.
- Vẫn giữ nguyên toàn bộ gain P/D, watchdog, bộ trộn, giới hạn cũ và
  khóa đầu ra động cơ trước khi AutoCal Ready.

Giới hạn 0.03 Nm nhỏ hơn mô-men bù khoảng 0.153 Nm trong log cũ. Vì vậy không
kỳ vọng bản thử đầu tiên tự xóa hết sai lệch -4 độ; mục tiêu là kiểm tra nhánh
I có đổi mô-men đúng hướng, làm sai lệch giảm và không gây dao động tăng.
Không tự tăng giới hạn/gain để ép góc về 0.

## Sau khi build đạt

Trước phép thử có lực đẩy, xác nhận góc đo phù hợp với góc thân thật và mô-men
phản hồi chống lại chiều nghiêng trên trục đang thử. Không dùng I để bù một
góc IMU đo sai, thứ tự động cơ sai hoặc cơ cấu treo bị kẹt.

Chỉ chạy một Stabilizer. Dùng Mission Server như trước, tải/chạy RollI khi
chuẩn bị thử trên giá. Throttle 0, Disarm, giữ đứng yên cho AutoCal Ready=1;
kiểm tra lại tốc độ góc trước khi Arm.

Giữ lệnh roll/pitch ở giữa. Chỉ thử trong phạm vi kiểm soát của giá, không
lặp lại các đoạn góc vượt 60 độ trong log trước. Khi `RollI Active=1`, log
cho biết I có làm góc giảm hay không. Nếu nghiêng/dao động tăng, Disarm và dừng
phép thử. Đây chưa phải xác nhận khả năng bay tự do.

Sau Disarm, Stop để hoàn tất các file trong
`saved_data/Stabilizer_QDrone_048378_RollI/`. Vẫn có Controller, Motors,
GyroCal như AutoCal; thêm `RollI_...mat` với biến `roll_i_data`:

| Hàng payload | Dữ liệu |
| --- | --- |
| 1 | Clock, s |
| 2..4 | Mô-men I roll/pitch/yaw, Nm |
| 5 | Nhánh I được cho phép, 0/1 |
| 6..8 | Sai số tốc độ góc, rad/s |
| 9 | AutoCal Ready |
| 10 | Cờ giới hạn mẫu trước |
| 11..13 | Mô-men PD gốc trước cộng I, Nm |

QUARC có thể thêm timestamp ở hàng đầu, khi đó file có 14 hàng. Log Controller
trước giới hạn nay chứa tổng PD+I; log RollI giữ riêng PD để đối chiếu.

## Trạng thái xác minh

Code Analyzer R2019a đã kiểm tra cú pháp. Các test thuật toán, Update Diagram
và build cần được thực thi trong MATLAB bằng lệnh setup ở trên; chưa chạy
được các bước này từ công cụ. Chưa có dữ liệu thực tế của RollI.
