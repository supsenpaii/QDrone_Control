# Kết nối QDrone với QGroundControl để xem cảm biến

**Bản cập nhật:** xem [FULL_TELEMETRY_VI.md](FULL_TELEMETRY_VI.md) để gửi toàn bộ trường cảm biến hiện có hoặc tạo nhánh telemetry từ Stabilizer với góc nghiêng, Arm/mode và lệnh động cơ. Phần dưới là quy trình dùng nguồn capture.

Cầu nối: `QDrone_Capture_IMU` hoặc `QDrone_Capture` trên drone → TCP 18910 → Python trên PC → MAVLink UDP 127.0.0.1:14550 → QGC trên cùng PC.

Bản capture gửi HEARTBEAT (1 Hz), HIGHRES_IMU (tối đa khoảng 20 Hz), BATTERY_STATUS, SYS_STATUS (1 Hz), NAMED_VALUE_FLOAT (2 Hz/trường). Gia tốc dùng m/s², gyro dùng rad/s, điện áp pin dùng mV trong MAVLink. Nếu gói nguồn có từ kế/áp suất/nhiệt độ thì cũng được gửi. Không gửi góc ATTITUDE, GPS, vị trí hoặc độ cao từ luồng capture. Bản tin chuẩn đổi trục Quanser FLU sang MAVLink FRD; named values giữ nguyên trục gốc.

QGC hiển thị vehicle generic, standby/disarmed của bộ chuyển đổi cảm biến; đây không phải trạng thái arm của model Stabilizer. Cầu nối chỉ đọc TCP từ drone, không chuyển lệnh QGC về drone. Parameters hiện chỉ có cấu hình bridge BRG_* chỉ đọc, chưa có gain PID. Danh sách mission rỗng, các lệnh bay không được chuyển tiếp. Tiêu chí kết nối là dữ liệu thật tăng trong MAVLink Inspector.

## 1. Chuẩn bị

- Disarm, tháo cánh khi kiểm tra trên bàn.
- PC và drone cùng mạng; cấu hình hiện tại: PC 192.168.2.5, drone 192.168.2.52. Giữ subnet mask theo mạng QDrone đang dùng; nếu dùng /24 thì là 255.255.255.0.
- Dừng các model Stabilizer, Read_IMU, Read_Cameras và Mission Server đang chạy trước khi chuyển sang model capture. Chỉ một model dùng phần cứng QDrone.
- Đóng dashboard capture/height observer và các ứng dụng đang đọc TCP 18910 để dành luồng đó cho cầu nối.
- Cài và mở QGroundControl: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

PowerShell:

```powershell
cd D:\QDrone\QDrone_Control
Test-Connection 192.168.2.52 -Count 2
```

## 2. Cài thư viện Python một lần

Trên máy này Python thật ở đường dẫn dưới; launcher `py` đang trỏ đến bản Microsoft Store không chạy được trong phiên kiểm tra. Môi trường `.venv` đã được tạo; cài thư viện bằng:

```powershell
cd D:\QDrone\QDrone_Control
& '.\qgc_bridge\.venv\Scripts\python.exe' -m pip install -r .\qgc_bridge\requirements.txt
```

Nếu `.venv` chưa có hoặc bạn chuyển sang máy khác, tạo trước bằng Python 3.10 trở lên:

```powershell
& 'C:\Users\user\AppData\Local\Programs\Python\Python310\python.exe' -m venv .\qgc_bridge\.venv
```

Nếu pip báo WinError 10013 hoặc không truy cập được mạng, chạy lệnh cài trong PowerShell của bạn có quyền truy cập Internet. Ở lần cập nhật hiện tại đã xác nhận pymavlink 2.4.50 có trong `.venv` và chạy được các test serialization/loopback.

## 3. Chạy model cảm biến trên drone

Trong MATLAB/QUARC đang dùng:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'capture_tool'));
open_system('QDrone_Capture_IMU.slx');
```

Chọn **QUARC > Build**, đợi build/download thành công, rồi **QUARC > Start**. Model IMU có sẵn, không cần chạy script tạo lại model. Không mở `open_capture_tool`/`start_qdrone_capture` vì chúng tạo thêm client TCP. Có thể dùng `QDrone_Capture.slx` thay thế nhưng camera làm tăng tải.

Sau khi model chạy, kiểm tra từ PowerShell:

```powershell
Test-NetConnection 192.168.2.52 -Port 18910
```

Cần `TcpTestSucceeded : True`. Lệnh kiểm tra tạm mở rồi đóng kết nối; đợi vài giây trước khi chạy cầu nối.

## 4. Cấu hình QGC

Mở **Application Settings > Comm Links** và bật **UDP AutoConnect** nếu có. QGC thường lắng nghe UDP 14550 mặc định.

Nếu không tự nhận, tắt UDP AutoConnect rồi tạo link thủ công để tránh hai link cùng chiếm cổng:

- Add / Add New Link, Name: `QDrone monitor`.
- Type: `UDP`.
- Listening/Local Port: `14550`.
- Server/Remote Hosts để trống, lưu rồi Connect.

QGC nhận dữ liệu từ Python trên chính PC, vì vậy không nhập IP drone vào link UDP này. Không chọn TCP 18910 trong QGC: luồng QUARC đó chưa phải MAVLink.

## 5. Chạy cầu nối

PowerShell, giữ cửa sổ mở:

```powershell
cd D:\QDrone\QDrone_Control
& '.\qgc_bridge\.venv\Scripts\python.exe' .\qgc_bridge\qdrone_to_qgc.py
```

Khi có dữ liệu thật, cửa sổ sẽ in `LIVE packets=... target_s=... battery=... V`. Bridge chỉ phát HEARTBEAT sau khi nhận gói thật, và ngừng phát khi không còn gói hợp lệ trong 2 giây. QGC sẽ báo mất liên kết sau khoảng timeout riêng của nó.

Nếu QGC nằm trên máy khác, thay địa chỉ bằng IPv4 máy chạy QGC:

```powershell
& '.\qgc_bridge\.venv\Scripts\python.exe' .\qgc_bridge\qdrone_to_qgc.py --qgc-ip 192.168.2.5
```

Máy QGC phải cho phép UDP 14550 trên mạng tương ứng.

## 6. Xác nhận

Vào **Analyze Tools > MAVLink Inspector** (tên menu có thể khác theo phiên bản). Chọn System 1 / Component 1:

- HEARTBEAT tăng khoảng 1 Hz.
- HIGHRES_IMU tăng đến khoảng 20 Hz; mở xacc/yacc/zacc, xgyro/ygyro/zgyro. Gia tốc khi đứng yên thường có độ lớn gần 9.81 m/s²; dấu theo trục cảm biến. Xoay nhẹ drone bằng tay để xem gyro thay đổi.
- BATTERY_STATUS tăng khoảng 1 Hz nếu điện áp hợp lệ. `voltages[0]` là tổng điện áp pin theo mV; không có % pin/dòng điện.

Màn hình Fly chưa có góc nghiêng, vị trí bản đồ hoặc độ cao thật. Không dùng các nút Arm/Takeoff/Mission của QGC với bản monitor này.

## Lỗi thường gặp

| Hiện tượng | Kiểm tra |
| --- | --- |
| Không ping được drone | Nguồn drone, Wi-Fi/Ethernet, IP/subnet PC |
| TCP 18910 không mở | Đã Start đúng model capture chưa; model Stabilizer gốc không có cổng này |
| TCP mở nhưng không có LIVE | Đóng client capture khác; kiểm tra model có chạy và gửi dữ liệu; model phải dùng schema capture 30 doubles little-endian |
| Có LIVE nhưng QGC không nhận | QGC lắng nghe UDP 14550, đúng --qgc-ip, chỉ một QGC/link chiếm cổng |
| Có vehicle nhưng thiếu parameters | Bản monitor chưa có parameter/mission protocol; kiểm tra Inspector |
| Pymavlink chưa cài | Chạy lệnh pip trong bước 2 bằng đúng Python `.venv` |

Dừng bằng Ctrl+C ở cầu nối, rồi QUARC > Stop model capture. Muốn quay lại bay bằng Simulink, dừng capture trước khi chạy lại Mission Server và Stabilizer.

## Tài liệu giao thức

- QGC Comm Links: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html
- QGC MAVLink Inspector: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html
- MAVLink message fields: https://mavlink.io/en/messages/common.html

Người dùng báo nguồn capture đã hiện dữ liệu trên QGC. Nguồn flight mới chưa xác minh với model/drone thật. Xem validation.log để biết kiểm tra cục bộ đã chạy.
