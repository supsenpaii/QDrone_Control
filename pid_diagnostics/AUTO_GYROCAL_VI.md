# Hiệu chuẩn gyro theo từng lần chạy

Đây là bước sửa offset cảm biến, chưa phải autotune PID và chưa xác nhận hết
nguyên nhân lệch góc. Không dùng lại hằng số bias của phiên chạy cũ.

## Tạo và build

Đóng model AutoCal nếu đã mở. Trong MATLAB:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath('pid_diagnostics');
clear setup_auto_gyro_calibration qdrone_gyrocal_step;
setup_auto_gyro_calibration(true);
```

Script chạy thử thuật toán với các tình huống đứng yên, Arm sớm, mẫu NaN,
nhiễu quá lớn, bias đóng băng khi Arm. Sau đó tạo
`Stabilizer_QDrone_048378_AutoCal.mdl` từ bản chẩn đoán đã lưu trên đĩa,
thêm hiệu chuẩn và khóa Arm vật lý, Update Diagram, kiểm tra các kết nối
và build cục bộ. Không tự download, Start hoặc Arm. Nếu file AutoCal đã có,
script lưu backup có thời gian trước khi tạo lại.

Không sửa model gốc, PID_Diag hoặc GyroCal cũ. Gain điều khiển, bộ trộn,
giới hạn mô-men/động cơ và watchdog giữ nguyên. Source mới được tạo bằng API
Simulink để tránh lỗi thứ tự khai báo đường nối trong file MDL.

## Cách chạy sau khi build đạt

1. Dừng mọi bản Stabilizer khác đang truy cập drone. Chạy Mission Server.
2. Mở model AutoCal, QUARC Build để tải lên target khi chuẩn bị thử.
3. Throttle=0, Disarm; để drone hoàn toàn đứng yên và Start model.
4. Trong `PHYSICAL PLANT w/ LINEAR THRUST`, xem hai Display:
   `Calibration Progress 0 to 1` và `Calibration Ready 0 or 1`.
5. Sau 5 giây dữ liệu đạt yêu cầu, Ready chuyển từ 0 sang 1. Nếu drone rung
   hoặc bị dịch chuyển, cửa sổ hiệu chuẩn bị loại và lấy lại. Không Arm để
   vượt qua bước này. Arm sớm khi chưa Ready làm bộ đếm trở về 0 và đầu ra
   động cơ vẫn bị khóa ở trạng thái Disarm.
6. Khi Ready=1, vẫn giữ Disarm để kiểm tra `Rate Estimate (deg/s)` có giá trị
   trung bình gần 0 khi drone đứng yên. Chưa thử có lực đẩy nếu bước này sai.
7. Chỉ sau khi kiểm tra đúng mới thực hiện phép thử trên giá trong phạm vi
   kiểm soát. Bias giữ nguyên sau Ready, không học chuyển động lúc Arm.

Mỗi lần khởi động model, bộ đếm và bias bắt đầu lại. Thuật toán kiểm tra
5000 mẫu ở 1 ms: Disarm, dữ liệu hữu hạn, gyro mỗi trục dưới 5 deg/s,
độ lớn gia tốc trong 8..11.5 m/s^2; độ lệch chuẩn cửa sổ của gyro dưới
0.75 deg/s và gia tốc dưới 0.20 m/s^2. Các tiêu chí này không thay thế việc
người vận hành bảo đảm drone thực sự đứng yên; chuyển động đều chậm vẫn có
thể bị nhận nhầm là bias.

Chỉ Arm ở nhánh phần cứng được chặn đến khi Ready; lệnh Arm gốc trong
controller vẫn là yêu cầu từ Mission. LED Arm và Arm trong log Motors là
Arm hiệu lực sau khóa. Arm trong log Controller là yêu cầu trước khóa.

## Log để xác minh

Thư mục `saved_data/Stabilizer_QDrone_048378_AutoCal/` gồm Controller,
Motors và GyroCal. File GyroCal có biến `gyro_cal_data`, payload:

| Hàng | Dữ liệu |
| --- | --- |
| 1 | Clock, s |
| 2..4 | Gyro thô trước bù, rad/s |
| 5..7 | Gyro đã bù, rad/s |
| 8..10 | Bias đo được, rad/s |
| 11 | Ready |
| 12 | Tiến độ 0..1 |
| 13 | Arm yêu cầu |
| 14 | Arm hiệu lực tại nhánh động cơ |

QUARC có thể thêm timestamp ở hàng đầu như các log cũ; khi đó file có 15 hàng.
Ghi 500 Hz với decimation=2. Dừng model sau Disarm để hoàn tất file.

## Trạng thái kiểm tra

Đã kiểm tra cú pháp ba file MATLAB bằng Code Analyzer R2019a. Chưa thực thi
các test MATLAB, Update Diagram hoặc build AutoCal từ công cụ; các bước này
sẽ chạy trong lệnh setup ở trên. Bản mới chưa được xác nhận vận hành thực tế.
Kết quả kiểm tra cần có `PASS: AutoCal diagram, gyro routing, motor Arm gate
and calibration telemetry` và `PASS: AutoCal local build` trước khi sử dụng.
