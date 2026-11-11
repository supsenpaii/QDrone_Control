# Tool thu thập QDrone – chỉ đọc, không điều khiển động cơ

## Đã triển khai

- Model: D:\QDrone\QDrone_Control\QDrone_Capture.slx
- Không có HIL Write; PWM/digital/analog output channels để trống; mọi cờ ghi đầu ra khi khởi tạo/dừng đều tắt.
- Đọc analog 0–4 (pin ở kênh 4), gyro, accelerometer, magnetometer, áp suất và nhiệt độ barometer.
- Telemetry yêu cầu chu kỳ 0,001 s. Đây là chu kỳ đọc; không có nghĩa mọi cảm biến có mẫu vật lý mới ở 1 kHz.
- Optical flow tính từ camera hướng xuống mỗi 0,033 s.
- Mặc định ghi ảnh hướng xuống mỗi 0,099 s (~10,1 FPS); RGB, Depth, IR trái/phải mỗi 0,495 s (~2,02 FPS).
- Ảnh lưu nguyên pixel (uint8 hoặc uint16), không nén mất dữ liệu. Số frame và timestamp nằm trong frames.csv.
- Dashboard, ảnh PNG xem nhanh và latest.json cập nhật khoảng 1 lần/giây.
- Mẫu TCP bị mất, dữ liệu cũ, lỗi kết nối và trạng thái không có dữ liệu được báo riêng.

## Phần chưa có

Không có cảm biến range trong model đã kiểm tra. Không đo RPM động cơ hoặc lệnh FrSky trong model sensor-only.
Không suy ra vị trí theo mét, không tune gain bay, không tự bay hoặc giữ vị trí.
Timestamp camera là thời điểm đọc bằng clock của target, KHÔNG phải timestamp exposure phần cứng.
Ảnh Depth giữ nguyên uint16; chưa ghi depth scale/intrinsics, vì vậy không tự coi giá trị pixel là mét.
Năm camera dùng chung clock phần mềm nhưng không được đồng bộ exposure phần cứng.

## Chạy lần đầu

1. Hạ drone, Disarm, tháo cánh. Dừng mọi target Stabilizer, Read_Cameras, Read_IMU và Mission Server đang chạy.
2. Trong MATLAB:

   cd('D:\QDrone\QDrone_Control');
   addpath(fullfile(pwd,'capture_tool'),fullfile(pwd,'optical_flow'));
   rec = open_capture_tool;

3. Trong cửa sổ QDrone_Capture, chọn QUARC > Build. Build có thể tải file lên drone nhưng không phải là Start.
4. Chọn QUARC > Start. Dashboard sẽ tự kết nối telemetry TCP 18910 và ảnh 18911–18915 ở IP 192.168.2.52.
5. Đợi có dữ liệu. Đặt drone đứng yên; kiểm tra telemetry age thấp, số mẫu tăng, camera hiển thị ảnh mới.
6. Bấm STOP RECORDING khi xong; sau đó QUARC > Stop model QDrone_Capture.

Không chạy model thu thập đồng thời với model bay; HIL được đặt exclusive để phát hiện dùng trùng board.
Nếu cần đổi IP:
   build_qdrone_capture(true,'192.168.2.52');
   rec = open_capture_tool('192.168.2.52');
Đóng QDrone_Capture trước khi tạo lại sơ đồ bằng build_qdrone_capture.

## Giảm tải Wi-Fi / tùy chỉnh camera

Mặc định toàn bộ ảnh có thể dùng khoảng 7 MB/s và khoảng 400 MB/phút trên đĩa. Đây là ước tính; xem dung lượng thực trong latest.json.
Nếu Wi-Fi không đủ, hãy dừng target và đóng model, rồi chỉ thu camera hướng xuống:
   build_qdrone_capture(false);
Sau đó Build/Start lại. Các cổng camera không có sẽ hiển thị chưa kết nối; telemetry vẫn được ghi.
Muốn ghi camera hướng xuống đủ 30,3 FPS, cấu hình:
   build_qdrone_capture(true,'192.168.2.52',0.033);
Phải kiểm tra dropped samples và tốc độ ghi đĩa. Camera IR/RGB/Depth vẫn lấy ~2,02 FPS.
Chỉ ghi telemetry (không kết nối luồng ảnh):
   rec = open_capture_tool('192.168.2.52',false);

## Bài thử cầm tay

Mỗi bài 10–20 giây. Chọn tên bài trong dropdown rồi bấm MARK STAGE ngay trước khi làm.
- stationary: đặt trên mặt phẳng, giữ yên 20 giây.
- translate_forward: giữ hướng và độ cao, dịch trước/sau theo đoạn đã đánh dấu.
- translate_right: giữ hướng và độ cao, dịch trái/phải.
- rotate_roll, rotate_pitch, rotate_yaw: xoay từng trục chậm, hạn chế dịch chuyển tâm camera.
- raise_lower: nâng/hạ chậm ở cùng một khu vực.
- lighting: giữ nguyên động tác nhưng thay điều kiện sáng hoặc nền.
Điền Distance m và Height m theo thước đo khi có; để NaN nếu chưa biết. Đừng nhập giá trị ước đoán làm dữ liệu chuẩn.
Bấm MARK STAGE khi quay về stationary hoặc end để tạo mốc kết thúc bài trước.
Các đánh giá trên dashboard là gợi ý kiểm tra dữ liệu, không chứng nhận an toàn bay.

## Tôi và trợ lý xem trực tiếp như thế nào?

Dashboard chạy cục bộ và tự cập nhật. Trợ lý có thể đọc file khi đang tương tác:
- capture_sessions\active_session.json: đường dẫn phiên hiện tại.
- <session>\latest.json: số đo mới nhất, tuổi dữ liệu, quality, lỗi và số mẫu mất.
- <session>\latest_camera_1.png: ảnh camera xuống. Camera 2–5: RGB, Depth, IR trái, IR phải.
Bạn nhắn “đọc dữ liệu hiện tại” hoặc “đánh giá bài stationary”; trợ lý đọc snapshot/log và trả nhận xét.
Trợ lý không có kết nối video liên tục mặc định và không tự theo dõi sau khi cuộc tương tác kết thúc.

## Dữ liệu lưu

Mỗi phiên nằm trong capture_sessions\YYYYMMDD_HHMMSS_<id>:
- session.json: cấu hình, giới hạn timestamp và các thành phần chưa có.
- telemetry.csv: toàn bộ gói telemetry nhận được; host_receive_utc_s là giờ nhận, không cùng clock với target_s.
- events.csv: mốc bài thử và khoảng cách/độ cao đo bằng tay.
- camera_1.bin ... camera_5.bin: pixel thô, thứ tự cột theo MATLAB.
- frames.csv: id, seq, target_s, host_receive_utc_s, epoch, byte_offset, payload_bytes và kích thước ảnh.
- latest.json, latest_camera_N.png: snapshot để theo dõi.
- report.json và REPORT.txt: báo cáo khi dừng ghi.
- VALIDATION_ONLY.txt nếu là phiên dữ liệu giả phục vụ kiểm thử.

Đọc lại một frame:
   image = read_qdrone_frame(rec.Session,1,1);
   imshow(image);
Tạo lại báo cáo sau khi dừng:
   qdrone_capture_report(rec.Session);

## Xác minh

Xem capture_tool\validation.log. Bài kiểm tra gồm thuật toán optical flow, phân mảnh TCP,
TCP loopback thật trên localhost, pixel ảnh xám/RGB/depth, model update, không có HIL Write,
sinh mã C cho packet, dashboard mất kết nối và lưu/đọc lại ảnh thô.
Chưa kiểm tra lưu lượng năm camera trên Wi-Fi/drone và chưa kiểm thử bay.
