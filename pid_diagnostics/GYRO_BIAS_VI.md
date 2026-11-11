# Bản thử bù gyro, PID giữ nguyên

Model: `Stabilizer_QDrone_048378_GyroCal.mdl`.
Nguồn: bản `Stabilizer_QDrone_048378_PID_Diag.mdl` đã kiểm tra kết nối và build.
Không sửa file Stabilizer gốc hoặc bản chẩn đoán.

Thêm một block Bias tên `Gyro Bias Compensation` ngay sau ngõ gyro của Demux
trong `PHYSICAL PLANT w/ LINEAR THRUST`. Mọi nhánh gyro cũ đi qua block này,
bao gồm tín hiệu dùng cho điều khiển và ước lượng. Không thay gain PID,
bộ trộn, giới hạn đầu ra, watchdog hoặc tạo tín hiệu kích thích.

Giá trị bias ước tính từ log Controller ngày 08/11/2026 13:38:32, đoạn 40–50 s,
Arm=0, throttle=0:

| Trục | Bias đo, deg/s | Giá trị cộng vào gyro, rad/s |
| --- | ---: | ---: |
| X | -1.3207397461 | +0.0230512571313 |
| Y | -0.0612182617 | +0.00106846022934 |
| Z | -0.1445556641 | +0.00252297229030 |

Công thức: `gyro_corrected = gyro_raw - estimated_bias`.
Đây là ước tính ban đầu từ đoạn log ít chuyển động, chưa phải hiệu chuẩn được
xác minh độc lập bằng trạng thái vật lý đứng yên. Cần kiểm tra lại ở trạng thái
Disarm, động cơ dừng, thân đứng yên trước phép thử có lực đẩy. Bias cũng có thể
thay đổi theo nhiệt độ và lần khởi động cảm biến.

## Kiểm tra và build

Đóng model GyroCal trước khi chạy:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath('pid_diagnostics');
clear check_pid_diagnostics check_gyro_bias;
check_gyro_bias;       % nguồn nối, giá trị bù, nhánh log, Update Diagram
check_gyro_bias(true); % build cục bộ, không download hoặc Start
```

Sau khi kiểm tra và build thành công, dùng bản GyroCal thay bản Stabilizer
đang chạy; chỉ chạy một model truy cập phần cứng. Mở Scope
`PHYSICAL PLANT w/ LINEAR THRUST / Rate Estimate (deg/s)` để xem gyro đã bù.
Khi Disarm và hoàn toàn đứng yên, giá trị trung bình tốc độ góc phải gần 0;
còn nhiễu tức thời là bình thường. Nếu offset không giảm, chưa tiếp tục thử
có lực đẩy; kiểm tra lại nguồn tín hiệu và lấy log đứng yên mới.

Log của bản này ở `saved_data/Stabilizer_QDrone_048378_GyroCal/`, cùng định dạng
Controller/Motors như bản chẩn đoán. Bước tiếp theo là đối chiếu log trước/sau;
bù gyro không được coi là đã xử lý toàn bộ sai lệch roll -3 deg.

Đã kiểm tra cấu trúc file và phát lại phép bù trên log cũ. Chưa xác nhận
Update Diagram, build hoặc chạy thực tế cho bản GyroCal từ công cụ.
