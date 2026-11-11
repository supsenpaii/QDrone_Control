# Telemetry QDrone → QGC từ bộ điều khiển thật

Có hai nguồn, không chạy đồng thời hai model trên cùng drone:

| Nguồn | Model | TCP | Lệnh Python |
| --- | --- | --- | --- |
| Cảm biến | QDrone_Capture_IMU hoặc QDrone_Capture | 18910 | `--source capture` (mặc định) |
| Bộ điều khiển bay | Bản sao Stabilizer có hậu tố _QGC | 18920 | `--source flight` |

QGC vẫn nhận UDP 14550. Server Addresses để trống nếu chạy QGC và Python cùng PC.

## Dùng ngay với model capture đang chạy

Ctrl+C cầu nối cũ, chạy lại script đã cập nhật:

```powershell
cd D:\QDrone\QDrone_Control
& '.\qgc_bridge\.venv\Scripts\python.exe' .\qgc_bridge\qdrone_to_qgc.py
```

Bản cập nhật đưa mọi trường có giá trị hữu hạn của gói capture sang NAMED_VALUE_FLOAT (2 Hz) và ghi vào telemetry log của QGC. HIGHRES_IMU giờ có thêm từ kế, áp suất và nhiệt độ nếu model thực sự đọc được. QDrone_Capture_IMU có thể không có nhiệt độ/flow; những trường không có trong model không được ước lượng thêm. Các giá trị flow zero/valid=0 của bản IMU không biểu thị phép đo optical flow hợp lệ.

Capture vẫn không có trạng thái Arm/mode hay góc ước lượng của Stabilizer. Muốn thấy attitude và trạng thái bay thật, chuyển sang nguồn flight dưới đây.

## Tạo model telemetry của Stabilizer

Thực hiện khi các model drone đã Stop, drone Disarm và tháo cánh. Lưu model nguồn trước để bản sao dùng đúng gain bạn đang dùng. Script giữ mọi kết nối điều khiển gốc, thêm nhánh quan sát với Rate Transition 50 Hz, dùng cổng TCP riêng. Không thêm đầu đọc HIL, không thay bộ ước lượng/gain/mixer/watchdog. Có thêm tải tính toán và truyền thông nên phải kiểm tra overrun thực tế trước khi dùng trong bài thử có lực đẩy.

Trong MATLAB R2019a/QUARC:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'qgc_bridge'));
setup_qgc_telemetry('Stabilizer_QDrone',true);
```

Script tạo `Stabilizer_QDrone_QGC.mdl` sau khi Update Diagram kiểm tra kích thước gói 59 doubles bằng Signal Specification và kiểm tra các kết nối gốc. Không gọi `feval(...,'compile')` ở normal simulation vì có thể kích hoạt HIL và gặp giới hạn license QUARC Home. `true` còn build executable cục bộ với DownloadToTarget=off; không tự Start hoặc Arm. File nguồn giữ nguyên. Bản cũ cùng tên được backup trước khi lưu bản mới. Nếu không có PASS thì không chuyển sang chạy bản mới; gửi lỗi MATLAB để sửa.

Nếu dùng AutoCal hoặc một bản MDL khác cùng cấu trúc:

```matlab
setup_qgc_telemetry('Stabilizer_QDrone_048378_AutoCal',true);
```

Model đầu ra sẽ là `Stabilizer_QDrone_048378_AutoCal_QGC.mdl`. Nhánh Arm được lấy tại input plant, vì vậy bản AutoCal vẫn giữ cổng Arm/calibration hiện có. Nếu cấu trúc các block nguồn khác với script, setup phải báo lỗi thay vì đoán tín hiệu.

## Chạy nguồn flight

1. Ctrl+C Python cũ; Stop model capture và các client thu thập.
2. Chạy Mission Server trên PC như quy trình bay QUARC hiện tại, giữ Disarm. Bridge không thay thế Mission Server/FrSky.
3. Mở `Stabilizer_QDrone_QGC.mdl`, chọn QUARC > Build để download khi sẵn sàng, rồi QUARC > Start. Chỉ một Stabilizer dùng phần cứng.
4. Trong PowerShell:

```powershell
cd D:\QDrone\QDrone_Control
& '.\qgc_bridge\.venv\Scripts\python.exe' .\qgc_bridge\qdrone_to_qgc.py --source flight
```

5. Giữ link UDP QGC ở 14550. Cần thấy `LIVE ... schema=2 ...`.
6. Mở MAVLink Inspector để kiểm tra ATTITUDE, HIGHRES_IMU, BATTERY_STATUS, SYS_STATUS, ACTUATOR_OUTPUT_STATUS và các NAMED_VALUE_FLOAT. Fly có thể dùng ATTITUDE để hiển thị nghiêng; lệnh động cơ và tín hiệu riêng xem trong Inspector. Flight mode là mã QUARC gốc; giao diện mode generic có thể không có tên mô tả như PX4.

## Dữ liệu gửi

| MAVLink | Tần số | Nguồn |
| --- | --- | --- |
| HEARTBEAT | 1 Hz | Flight: Arm tại plant, mode từ Mission, critical nếu model báo low battery/sensor failure/communication loss. Capture: trạng thái bộ chuyển đổi, không có Arm thật |
| ATTITUDE | 20 Hz | Góc và tốc độ góc ước lượng của Stabilizer, chỉ flight |
| HIGHRES_IMU | 20 Hz | Gia tốc, gyro; mag/pressure/temp chỉ khi gói nguồn có giá trị |
| BATTERY_STATUS | 1 Hz | Điện áp pin đo. Dòng, dung lượng, % pin để unknown |
| SYS_STATUS | 1 Hz | Cảm biến gyro/accel, điện áp; lỗi cảm biến tổng nếu model báo. Chưa đo CPU load |
| ACTUATOR_OUTPUT_STATUS | 20 Hz | Bốn lệnh normalized sau nhân Arm, không phải RPM đo. Các kênh khác unused |
| NAMED_VALUE_FLOAT | 2 Hz/trường | Mọi số liệu hữu hạn còn lại: low_batt, sens_fail, comm_loss, arm, mode, stop_cmd, thrust_N, lệnh attitude/rate, mô-men, motor0..3, duty0..3, tín hiệu IMU/estimator và timestamp |
| STATUSTEXT | Khi trạng thái đổi | Thông báo nguồn, Arm/mode/fault |

Flight có 59 doubles: framing/time 4; battery/flags/Arm/mode/stop/throttle 8; command giới hạn 3; estimated attitude/rate/angular acceleration 9; gyro/accel 6; motor normalized 4; PWM raw 4; generalized force 4; torque giới hạn 3; rate command giới hạn 3; torque input 3; attitude command input 3; mag/pressure/temp 5 unknown. Tổng 59. Chi tiết tên/index trong flight_schema.json.

Stabilizer hiện tại không đọc mag/barometer nên 5 trường cuối là NaN, không được gửi như phép đo. Cầu nối không tự mở thêm HIL song song. Không ghép dữ liệu từ hai model cùng truy cập board.

## Hệ trục và đơn vị

Quanser dùng trục thân x trước, y trái, z lên (FLU). Với bản model gốc đang kiểm tra, các tín hiệu body được xuất trực tiếp theo trục đó. MAVLink dùng FRD cho vector thân: đổi `(x,y,z)` thành `(x,-y,-z)` khi phát HIGHRES_IMU. ATTITUDE đổi `(roll,pitch,yaw)` thành `(roll,-pitch,-yaw)` và rates tương tự. Nếu bạn đã tự sửa hệ trục trong model nguồn, phải cập nhật phép đổi tương ứng trước khi sử dụng.

NAMED_VALUE_FLOAT luôn giữ hệ trục/đơn vị gốc của QUARC để đối chiếu Scope. Mag T → gauss và pressure Pa → hPa chỉ trong bản tin chuẩn MAVLink. `duty0..3` giữ nguyên đơn vị HIL Write của model, không suy thành microsecond/RPM. Yaw ước lượng có thể trôi, không phải heading true north. Không có GPS, position hoặc altitude theo mét trong gói flight hiện tại.

Nguồn: [Quanser QDrone](https://docs.quanser.com/quarc/documentation/qdrone.html), [Quanser standard axes](https://docs.quanser.com/quarc/documentation/quarc_using_devices_standard_axes.html), [MAVLink common](https://mavlink.io/en/messages/common.html).

## Parameters và điều khiển từ QGC

QGC có thể tải các parameter BRG_SCHEMA, BRG_SYSID, BRG_TCP_PORT, BRG_READONLY, BRG_IMU_HZ, BRG_NAMED_HZ. Đây là cấu hình thực tế của cầu nối, chỉ đọc; sửa sẽ được trả về giá trị hiện tại. Chúng không đại diện cho gain PID của drone. Muốn thay IP/rate/source thì đổi cấu hình script rồi khởi động lại, không dùng PARAM_SET.

Nếu QGC đứng ở `Waiting for vehicle parameters to download...`: dùng bản Python mới và khởi động lại cầu nối bằng Ctrl+C rồi chạy lại `--source flight`, không cần build model. Cầu nối gửi AUTOPILOT_VERSION và đủ 6 PARAM_VALUE có count/index ngay khi có dữ liệu thật, lặp lại mỗi 5 giây; vẫn trả lời yêu cầu list/read và retry. PowerShell in `PARAMS advertised: 6 BRG_* (read-only)` và `param_rx`/`param_tx` (số yêu cầu nhận/giá trị gửi, không phải xác nhận QGC đã tải thành công). Nếu QGC vẫn chờ, đóng/mở lại QGC khi cầu nối đang LIVE. Bản sửa đã qua loopback nhưng chưa xác minh trên giao diện QGC thật.

PID vẫn chỉnh ở Simulink; chưa có giao thức đọc/ghi gain QUARC qua MAVLink. Không phát danh sách PID tĩnh rồi gọi đó là parameter đang chạy. QGC nhận AUTOPILOT_VERSION của adapter generic; không giả làm PX4/ArduPilot. Danh sách mission hiện rỗng; upload/clear bị từ chối. Các lệnh bay qua COMMAND_LONG/COMMAND_INT bị trả UNSUPPORTED; các message điều khiển khác không được chuyển đến drone.

Đây là tích hợp telemetry cho bộ điều khiển QUARC, chưa thay thế một flight stack MAVLink đầy đủ. Muốn Arm/Takeoff/mission/PID online giống các drone PX4 cần triển khai thêm phần xử lý lệnh và liên kết thật vào bộ điều khiển.

## Xác minh và dừng

13 test Python đã qua: phân mảnh TCP, mã hóa/giải mã MAVLink thật, đơn vị/hệ trục, Arm/mode/flags, parameter chỉ đọc, danh sách parameter đầy đủ khi chưa có request và retry theo index, từ chối lệnh Arm, và tiến trình bridge TCP → UDP trên localhost. Test dùng gói tổng hợp ở cổng localhost riêng, không gửi dữ liệu test vào QGC đang dùng. Xem validation.log.

Người dùng đã chạy setup trong MATLAB sẵn có: Update Diagram, kiểm tra 59 doubles và local build Linux x64 thành công, tạo `Stabilizer_QDrone_QGC.rt-linux_x64` (195184 byte trong lần kiểm tra artifact). Không download/Start/Arm trong bước setup. GCC có các cảnh báo `may be used uninitialized` trong mã sinh cho sensor monitoring/communication; cấu trúc guard major-time-step tương tự cũng có trong mã model nguồn, nhưng chưa có xác minh runtime để kết luận mọi cảnh báo vô hại. Chưa xác minh end-to-end flight schema trên drone thật; kết nối capture do người dùng báo đã hoạt động. Tiếp tục kiểm tra telemetry trên bàn khi Disarm/tháo cánh.

Kiểm tra khi Disarm: xoay bằng tay, đối chiếu QGC attitude với Scope, kiểm tra chiều roll/pitch và trạng thái. Không dùng QGC Arm/Takeoff để kiểm tra. Dừng model thì dữ liệu phải ngừng, bridge ngừng heartbeat khi mẫu không tiến; không giữ vehicle online bằng dữ liệu cũ.

Kết thúc: Ctrl+C bridge, Stop Stabilizer_QGC, rồi Stop Mission Server. Có thể quay lại model nguồn bất kỳ lúc nào sau khi dừng bản QGC.
