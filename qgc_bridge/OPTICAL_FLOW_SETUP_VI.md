# Bước tiếp theo: xem optical flow live trên QGC

Script mới: `setup_qgc_optical_flow.m`. Đầu vào là bản đã lưu `Stabilizer_QDrone_QGC.mdl`; đầu ra là bản riêng `Stabilizer_QDrone_QGC_OF.mdl`. Chỉ sao chép subsystem `Optical Flow Monitor` từ `Stabilizer_QDrone_048378_OF_Ready.slx`; không lấy gain/bộ điều khiển cũ từ model OF thay cho bản QGC hiện dùng. Script kiểm tra các kết nối gốc vẫn giữ đúng nguồn, Update Diagram và có thể build cục bộ; không tự download, Start hoặc Arm.

## Tạo/build

Disarm, tháo cánh. Ctrl+C cầu nối Python, Stop Stabilizer QGC rồi Stop Mission Server. Save model QGC sau khi Stop để bản sao giữ đúng cấu hình bạn đang dùng. Đóng bản `_QGC_OF` nếu đã mở từ lần thử trước.

Trong MATLAB:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'qgc_bridge'));
setup_qgc_optical_flow(true);
```

Cần PASS cho Update Diagram và local build. Script dùng bản source đã lưu trên đĩa và sẽ dừng nếu source đang có thay đổi chưa Save. Không gọi normal-simulation compile. Camera vẫn là device 2, 640x480, chu kỳ 0.033 s theo nhánh OF đã có. Rate Transition đưa flow từ 33 ms về base 1 ms rồi vào gói telemetry 20 ms; số frame vật lý không tăng theo tần số gói.

## Chạy

1. Chạy lại Mission Server, giữ Disarm.
2. Trong MATLAB:

```matlab
open_system('Stabilizer_QDrone_QGC_OF.mdl');
set_param('Stabilizer_QDrone_QGC_OF','DownloadToTarget','on');
```

3. Trong cửa sổ model `_QGC_OF`, QUARC > Build để tải lên target rồi QUARC > Start. Không chạy cùng model Stabilizer/capture/Read_Cameras khác.
4. PowerShell:

```powershell
cd D:\QDrone\QDrone_Control
& '.\qgc_bridge\.venv\Scripts\python.exe' .\qgc_bridge\qdrone_to_qgc.py --source flight
```

QGC vẫn UDP 14550; drone vẫn TCP 18920. Python tự nhận schema 2 hoặc 3. Model mới phải cho `schema=3`, với `flow_du`, `flow_dv`, `quality`, `valid`, `age` trong dòng LIVE. `BRG_SCHEMA` cũng đổi thành 3 theo gói thật nhận được.

## Xem và kiểm chứng

Trong QGC MAVLink Inspector, NAMED_VALUE_FLOAT có thêm 8 tên: `flow_du`, `flow_dv`, `flow_qual`, `flow_valid`, `flow_dt`, `flow_age`, `textured`, `inliers`. Giá trị gửi khoảng 2 Hz/trường; flow nội bộ tính khoảng 30 Hz. Có thể mở subsystem `STABILIZER - QDRONE/Optical Flow Monitor` để xem ảnh và các display ngay trong Simulink.

- Đặt camera nhìn nền có họa tiết, đủ ánh sáng, tránh quá sát nền hoặc mặt trơn/phản chiếu.
- Giữ drone Disarm, dịch chuyển nhẹ bằng tay để xem du/dv thay đổi. Giữ hướng tương đối ổn định khi đánh giá tịnh tiến vì chưa bù xoay bằng gyro.
- Kiểm tra quality/valid/age đồng thời. Không suy `du=dv=0` thành đứng yên nếu valid=0.
- Nếu target báo overrun/watchdog/camera busy, dừng bài thử và kiểm tra console. Có thêm tải camera và xử lý ảnh.

Du/dv là **chuyển động ảnh pixel/frame**, không phải vận tốc m/s hoặc vị trí theo mét. Script không gửi OPTICAL_FLOW_RAD vì chưa hiệu chuẩn góc và bù xoay; dữ liệu pixel được giữ nguyên trong named values.

Đây là bước kiểm tra cảm biến, chưa có mode giữ vị trí. Sau khi flow live được xác minh, cần kiểm tra nguồn khoảng cách xuống sàn, hiệu chuẩn camera/IMU và đồng bộ, ước lượng vận tốc/vị trí, rồi mới xây vòng điều khiển và xử lý mất dữ liệu.

## Trạng thái xác minh

15 test Python đã qua, gồm schema 1/2/3, đơn vị/hệ trục, flow giữ đơn vị pixel, parameter BRG_SCHEMA=3, luồng TCP→UDP với dữ liệu tổng hợp tại cổng localhost riêng và ngừng phát khi dữ liệu cũ. Không gửi dữ liệu test đến QGC đang dùng.

Script MATLAB mới chưa được Update Diagram/build hoặc chạy trên target trong phiên trợ lý. MATLAB cần xác nhận ở bước Tạo/build. Log OF có sẵn ngày 08/11 ghi 8512 mẫu, 97.4% hợp lệ; đó là dữ liệu lịch sử, không phải xác minh model mới.
