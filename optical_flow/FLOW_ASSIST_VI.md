# Flow Assist thử nghiệm cho QDrone

Đây là bộ hỗ trợ giảm trôi bằng chuyển động ảnh đã bù xoay, không giữ một tọa độ theo mét. Chưa xác nhận bay. Ga và yaw giữ nguyên điều khiển hiện tại: không thêm giữ độ cao hoặc giữ hướng. Vòng I roll/yaw có sẵn được giữ theo bản nguồn đã lưu, không tự bật/tune.

**Cập nhật stage:** config(1)=0 khóa, =1 tác động điều khiển, =2 chỉ tính diagnostic khi Disarm (status9), không thay lệnh góc. Quy trình calibration/profile và ground verification mới ở `FA_FLIGHT_SETUP_VI.md`. Stage2 vẫn cần fx/fy/R/h thật. Phần mô tả confirmed bên dưới là tên cũ của trường stage.

## SB ba nấc — cấu hình đã xác định từ tay cầm

Người dùng đã xác nhận SB là `Host Game Controller/9` (Rz): lên=-1, giữa=0, xuống=+1. Bản nâng cấp SB đọc axis này, không dùng bảng button. Lên/giữa chọn Angle (mã0); xuống yêu cầu Flow Assist (mã2).

Để cập nhật Mission Server FA đang có, hạ cánh/Disarm, Stop cả hai model, Save Mission Server rồi chạy:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'optical_flow'));
clear setup_sb_flow_assist qdrone_sb_mode_step QDroneSBModeBuildable
setup_sb_flow_assist(true);
```

Script sao lưu `.sb.bak`, thay chart chọn mode, nối Rz và cờ dữ liệu mới, thêm display SB/status, kiểm tra kết nối cũ rồi build LOCAL. Không download/Start/Arm. Không sửa cấu hình camera hoặc Stabilizer. Không cần chạy lại toàn bộ `setup_flow_assist` để nâng cấp SB. Lần tạo model mới bằng setup toàn bộ cũng gọi bước nâng cấp SB này.

Sau PASS, trong khi vẫn Disarm, download và chạy Mission Server mới:

```matlab
mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
qc_download_model(mission);
qc_start_model(mission);
```

Giữ SB LÊN ít nhất 0.15 s trước, rồi gạt xuống giữ ít nhất 0.12 s. Display `FA SB Raw -1 Up 0 Middle 1 Down` phải lần lượt hiện -1/0/+1 và display mode là 0/0/2. Để thao tác dễ kiểm chứng, giữ mỗi nấc khoảng 1 giây. Nếu khởi chạy với SB đang xuống, mode vẫn0 cho đến khi đã gạt lên. Sau mất dữ liệu tay cầm hoặc đổi base mode cũng cần gạt lên lại. Không đổi base Angle sang Torque.

`FA SB Status First`, 6 phần tử: status, seenUp, freshAge, rawSB, encodedMode, downHeldSeconds. Status0=tắt hoặc base khácAngle, 1=chờ SB lên, 2=Angle đã sẵn sàng, 3=chờ xuống đủ lâu, 4=FA request, 5=không có dữ liệu tay cầm mới trong0.25s, 6=axis/clock/input không hợp lệ. Scope là axis thật; không suy tên nút từ màu đồ thị.

Trong source hiện tại, nhánh Rz cũ có nhãn Takeoff/Autoland tạo trigger[2], nhưng đường Arm/E-stop thực tế chọn trigger[1]/[3]. Bộ nâng cấp giữ nguyên các đường này và kiểm tra mapping; không thêm lệnh cất/hạ cánh. Nếu source đã được tùy biến để dùng trigger[2] cho mục đích khác, phải audit lại trước khi dùng SB.

9 native tests trên C dùng chung đã PASS: hướng ba nấc, chống bật khi khởi động ở nấc xuống, xác nhận nấc lên, debounce xuống, thoát ngay, mất dữ liệu và bắt buộc lên lại, giữ modeTorque, dữ liệu xấu và clockreset. DLL/MEX build MSVC2017 không warning. MATLAB wrapper/diagram/build bản SB chưa được chạy trong phiên trợ lý; setup sẽ kiểm tra trên MATLAB của người dùng. Bản `_FA` cũ trên đĩa và executable chưa tự đổi cho đến khi chạy script cập nhật.

**Mã2 là yêu cầu FA, chưa chứng minh bộ bù đang tham gia. Khóa hiệu chuẩn camera vẫn giữ nguyên.** Phần hướng dẫn button bên dưới chỉ áp dụng cho bản cũ chưa nâng cấp SB; helper button sẽ từ chối gán index khi đã có bộ chọn SB.

## Các file đã viết

- `qdrone_fa_core.c/.h`: một nguồn thuật toán dùng chung cho kiểm tra native, MEX và QUARC.
- `qdrone_fa_step.m`, `QDroneFlowAssistBuildable.m`: wrapper và đăng ký C khi codegen R2019a.
- `setup_flow_assist.m`: tạo cặp model riêng từ RYPI và Mission Server; kiểm tra các kết nối cũ, chỉ thay đường lệnh góc và giải mã mode có chủ đích.
- `flow_assist_config.m`: cấu hình mặc định khóa hiệu chuẩn, chưa điền fx/fy/độ cao tham chiếu.
- `test_fa_native.py`, `test_flow_assist.m`: kiểm tra C dùng thực tế; không mở camera/HIL, không gửi lệnh motor.

## Tạo và build

Sau khi hạ cánh/Disarm, Stop model và lưu bản RYPI cùng Mission Server đang dùng. Các script sao chép bản đã lưu; nếu source Dirty thì dừng để tránh mất thay đổi của bạn.

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'optical_flow'),fullfile(pwd,'pid_diagnostics'));
clear setup_flow_assist qdrone_fa_step QDroneFlowAssistBuildable
setup_flow_assist(true);
```

Kết quả dự kiến:

1. PASS kiểm tra MEX.
2. PASS update diagram và kiểm tra đường dây cho hai model.
3. Local build hai model; không download, Start hoặc Arm.

Tên model:

```text
Stabilizer_QDrone_QGC_OF_RYPI_FA
Mission_Server_QDrone_Manual_Flight_FrSky_FA
```

Trong lượt trợ lý, đã build DLL/MEX và chạy native tests. Chưa chạy được MATLAB Update Diagram/slbuild cho hai model mới; không coi các bước trên đã PASS cho đến khi MATLAB của bạn xác nhận. Nếu lỗi, gửi toàn bộ thông báo. Không chạy normal simulation compile HIL.

## Khóa và chọn công tắc

### Khởi chạy Mission Server sau local build

Setup để `DownloadToTarget=off`, vì vậy chỉ có file executable chưa đủ để nhấn Run trong Simulink External mode. Nếu thấy `Cannot start External mode simulation because target is not connected`, cần download và dùng QUARC Start để kết nối/chạy. Mission Server là target **Windows trên PC**, không phải Linux trên drone.

Sau khi Disarm và dừng Mission Server cũ, chạy trong MATLAB:

```matlab
cd('D:\QDrone\QDrone_Control');
mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
open_system(mission);
set_param(mission,'SimulationMode','external');
qc_download_model(mission);
qc_start_model(mission);
```

Hai lệnh cuối có download và Start **Mission Server**, khác với setup chỉ local build. Chúng không Arm, không Start Stabilizer. Nếu download/connect còn lỗi, lấy nguyên lỗi kèm `qc_get_target_uri(mission)`; không đổi target Mission Server sang IP drone. Khi chạy bằng giao diện, dùng QUARC > Start thay cho chỉ nhấn tam giác Run trước khi kết nối.

`FA Configuration` trong Stabilizer có confirmed=0 và fx/fy/h_ref=0; do đó không có lệnh bù. `FA Button Index` trong Mission Server mặc định 0; công tắc chọn Assist bị vô hiệu. Hai khóa này có mục đích khác nhau: một khóa dữ liệu hiệu chuẩn, một khóa ánh xạ công tắc chưa biết.

Trong Mission Server mới, `MISSION SERVER QDRONE MANUAL FLIGHT/FA Raw Buttons 1 to 32` hiển thị các nút được Host Game Controller báo về. Khi Disarm, gạt công tắc còn trống để xác định đúng index và trạng thái 0/1. Kiểm tra nó không dùng cho Arm/E-stop/Takeoff/Autoland hoặc chức năng khác. Không đoán số nút từ tên công tắc. Code không cho chọn index 1–2 (đang là E-stop/Arm). Các công tắc biểu diễn bằng axis thay vì button cần ánh xạ thêm sau khi đo, không được gán bừa vào index.

Sau khi xác định index, thay `FA Button Index` từ 0 thành index tương ứng trên BẢN MỚI khi model dừng; lưu/build theo quy trình. Display `FA Encoded Mode 0 Angle 1 Torque 2 Assist` phải cho 0 khi tắt, 2 khi bật, và vẫn giữ 1 nếu base mode là Torque. Không đổi base sang Torque để thử Flow Assist.

Có thể dùng helper mở đúng màn hình và xem cấu hình:

```matlab
addpath(fullfile(pwd,'optical_flow'));
configure_flow_assist_switch;
```

Khi đã xác định nút trống N qua display, Disarm, Stop cả hai model, Save Mission Server rồi chạy `configure_flow_assist_switch(N)`. N là số đo được, không phải số ví dụ. Helper sao lưu model và chỉ đổi `FA Button Index`; không thay calibration và không tự build/download/Start. Sau đó dùng QUARC Build với DownloadToTarget=on và QUARC Start cho Mission Server mới. Có thể quay về chưa gán bằng `configure_flow_assist_switch(0)`. Nếu không có phần tử button nào đổi nhưng axis trong JOYSTICK/Scope đổi, công tắc đang được biểu diễn bằng axis; bộ chọn hiện tại chưa hỗ trợ đường đó, cần xác định đúng axis và chức năng đang sử dụng trước khi thêm.

**Chỉ dùng cặp `_FA` với nhau khi đã cho phép mã mode=2.** Mission cũ chỉ gửi 0/1 nên Assist không được chọn. Stabilizer cũ hiểu mọi mode>0.5 là Torque; dùng Mission mới gửi 2 tới Stabilizer cũ sẽ chọn sai nhánh. Setup giữ `FA Button Index=0` để không phát mã 2 trước khi xác minh cặp model.

## Dữ liệu cần đo trước khi mở khóa hiệu chuẩn

`flow_assist_config.m` trả config 13 giá trị:

| Index MATLAB | Nội dung | Mặc định |
|---|---|---|
| 1 | confirmed: đã kiểm chứng intrinsics, mounting, dấu/timing và chiều bù | 0 |
| 2–3 | fx,fy ở ảnh gốc 640×480, pixel | 0,0 |
| 4 | Khoảng cách danh định dọc trục quang tới sàn, đo thật, m | 0 |
| 5 | Gain angular-image-rate → angle, s | 0.03 |
| 6 | Cap mỗi góc bù | 3 độ |
| 7 | Slew khi vào/bám hỗ trợ | 4 độ/s |
| 8 | Hằng số lọc | 0.10 s |
| 9 | Quality tối thiểu | 0.35 |
| 10 | Giới hạn tuổi dữ liệu | 0.10 s |
| 11 | Thời gian flow tốt liên tục trước khi tham gia | 0.35 s |
| 12 | Deadband của cả hai lệnh góc roll/pitch | 1 độ |
| 13 | Ngưỡng roll/pitch đo để cho hỗ trợ | 12 độ |

Camera-to-body rotation mặc định **chỉ là ví dụ** camera nhìn xuống, cạnh trên ảnh hướng về mũi: `[0 -1 0;-1 0 0;0 0 -1]`. Phải kiểm chứng trước khi confirmed=1. Camera đặt `[0.10;0;-0.03]` m theo người dùng, số này được dùng trong C.

Ở chế độ không có range, phần bù vận tốc do camera lệch CG sử dụng h_ref đo trước và vì vậy chỉ xấp xỉ khi độ cao thực thay đổi. Không thể tự kiểm tra drone còn ở đúng khoảng độ cao từ flow hiện tại. Bộ điều khiển không giả định h_ref là độ cao live. Bù xoay dùng trường flow gần tâm và tích phân gyro theo cửa sổ frame phần mềm; chưa có hardware exposure timestamp, chưa bù đầy đủ biến dạng ảnh/méo ống kính. Cần kiểm chứng khi dịch chuyển và xoay Disarm. Không bỏ khóa bằng cách điền tiêu cự/chiều camera phỏng đoán.

Chỉnh config trong file không tự đổi Constant của model đã tạo. Khi hiệu chuẩn xong, cập nhật hai Constant `FA Configuration` và `FA Camera To Body` trên bản `_FA` đang dừng bằng giá trị đã đo, rồi lưu/build lại. Setup mới cố ý yêu cầu config(1)=0 cho lần dựng đầu. Không gọi lại setup để ghi đè một bản đã chỉnh mà chưa lưu riêng các giá trị.

## Luật hỗ trợ và ưu tiên

Flow pixel/frame được chuẩn hóa bởi fx/fy và dt. Bù gyro và phần quay quanh CG để lấy proxy vận tốc ngang/độ cao, đơn vị 1/s; không phải m/s. Trục camera right/down/forward được đổi sang body FLU bằng R. Lệnh bù là roll=K*proxy_Y, pitch=-K*proxy_X, lọc và giới hạn. Không có tích phân ở vòng Assist nên không tích lũy góc để bù motor yếu hay offset cảm biến. Bộ ổn định bên trong và anti-windup RYPI được giữ nguyên.

- Chỉ hỗ trợ khi request=1, Arm thực, ga>=4 N, base Angle, hai cần ngang trong deadband, attitude trong ngưỡng và cả ba gyro<40 độ/s.
- Người lái đẩy một trong hai cần ngang: trả nguyên cả hai lệnh roll/pitch của người lái, không tiếp tục giữ bù cũ.
- Ga và yaw không được sửa. Quay yaw nhanh có thể làm gate tắt Assist để ưu tiên điều khiển tay.
- Flow hợp lệ phải có timestamp mới, dt hợp lý khớp gyro, quality/inliers đủ, dịch ảnh chưa tới biên tìm kiếm. Mất/già flow hoặc điều kiện không hợp lệ: bù=0, trở về lệnh góc; không tự hạ cánh hoặc giữ độ cao.
- Khi tắt/mất dữ liệu, ưu tiên trả tay ngay; có thể có bước thay đổi tối đa bằng cap bù 3 độ. Slew được áp dụng khi hỗ trợ đang hoạt động, không cố giữ bù mất tin cậy để làm mượt thoát mode.
- Muốn hãm ngang hiệu quả, vòng ổn định góc phải bám đủ tốt. Lệch roll/motor nghi yếu đã thấy trong log vẫn cần xử lý; Assist không chẩn đoán hoặc sửa phần cứng.

## Chẩn đoán và log

`FA Diagnostic Status First` hiển thị 16 trường, phần tử đầu: 0 tắt, 1 khóa/cấu hình sai, 2 gate/ưu tiên tay, 3 đang chờ flow tốt, 4 flow thiếu/sai/già, 5 đang tham gia, 8 dữ liệu gyro/clock/input không hợp lệ.

Payload log `flow_assist_data`: 65 giá trị + QUARC timestamp = 66 hàng trong MAT. Chỉ số zero-based sau khi đọc MAT:

| Hàng | Nội dung |
|---|---|
| 0 | QUARC timestamp |
| 1 | clock |
| 2–4 | pilot command |
| 5–7 | command sau Assist, trước limiter gốc |
| 8 | frame timestamp phần mềm |
| 9–16 | flow 8 trường gốc |
| 17–19 | gyro |
| 20–22 | attitude |
| 23–26 | Arm,throttle,base mode,request |
| 27–42 | diagnostic 16 trường |
| 43–55 | config |
| 56–64 | R column-major |
| 65 | raw encoded mode |

Diagnostic: status,request,gate,newframe,age,qualified_seconds,compensated_cam_x/y,filtered_body_x/y,angle_offset_roll/pitch,target_roll/pitch,config_ok,frame_dt.

QGC telemetry schema 3/cổng 18920 được giữ, chưa có nhãn mode Assist riêng trên QGC. Dùng display và log để xác nhận mode/hoạt động thực; QGC vẫn không gửi lệnh bay vào drone qua bridge. Cần kiểm tra target console không overrun sau khi thêm controller, dù chưa thêm camera thứ hai.
