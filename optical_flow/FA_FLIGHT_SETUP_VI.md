# Cấu hình Flow Assist từ số đo thật

Trạng thái: đã có bộ điều khiển giảm trôi, SB -1/0/+1 và chế độ kiểm tra Disarm. Chưa triển khai bay. Project chưa có calibration camera, hướng ảnh đã đo hoặc khoảng cách camera-sàn danh định. Không thay bằng số phỏng đoán. Không có range nên gain/bù lever-arm vẫn phụ thuộc độ cao; không phải Position Hold theo mét.

## 1. Kiểm tra cấu hình hiện có

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'optical_flow'));
configure_fa_flight;
```

Lệnh không đối thoại với drone, không thay model. Các giá trị đang0 sẽ được báo thiếu.

## 2. Tạo file cameraParams

### Thu ảnh gốc từng tư thế

Tháo cánh, Disarm và dừng Stabilizer/Mission trước khi dùng model Capture.
Chuẩn bị bàn cờ đen trắng in phẳng, ví dụ 9 × 7 ô vuông. Đo cạnh một ô
thực tế bằng mm sau khi in. Đặt bàn cờ trong tầm nhìn camera dưới, đủ sáng.

```matlab
cd('D:\QDrone\QDrone_Control');
addpath('capture_tool','optical_flow');
rec = open_capture_tool('192.168.2.52',true);
```

Build, Download và Start model `QDrone_Capture` bằng QUARC. Đợi ảnh camera
dưới cập nhật trên dashboard. Giữ bàn cờ yên ở mỗi tư thế và chạy một lần.

```matlab
save_fa_calibration_image(rec);
```

Đổi vị trí bàn cờ ở giữa ảnh, bốn góc, nghiêng theo hai hướng và thay đổi
khoảng cách trong phạm vi vẫn thấy rõ toàn bàn. Chụp 20–30 tư thế khác nhau.
Hàm kiểm tra ảnh mới, đúng camera 640 × 480 và phát hiện được bàn cờ trước
khi lưu PNG gốc. Ảnh có overlay hoặc ảnh chụp màn hình không dùng cho bước này.

Khi xong, bấm STOP RECORDING trên dashboard rồi Stop target Capture.

```matlab
imageFolder = fullfile(rec.Session,'calibration_images');
squareSize_mm = input('Nhap canh o vuong da do, don vi mm: ');
cameraParams = calibrate_fa_camera(imageFolder,squareSize_mm);
figure; showReprojectionErrors(cameraParams);
figure; showExtrinsics(cameraParams);
```

Kiểm tra các ảnh có sai số lớn, ảnh nhòe và độ phủ bàn cờ trước khi áp dụng.
Sau đó đo khoảng cách camera-sàn và xác định hướng phía trên ảnh theo thân.
Gửi kết quả hiệu chuẩn và số đo để cấu hình stage shadow ở mục 3.

Thu ít nhất15 (nên20–30) ảnh checkerboard rõ, nhiều góc và vị trí, từ đúng camera flow640x480; đo kích thước ô vuông. Không dùng ảnh từ camera PC hay RealSense thay cho OV7251. `cameraCalibrator` có sẵn trong MATLAB này để kiểm tra phân bố điểm và reprojection errors. Hoặc chạy helper offline:

```matlab
cameraParams=calibrate_fa_camera(imageFolder,squareSize_mm);
```

`imageFolder` là thư mục ảnh thật, `squareSize_mm` là cạnh ô vuông đo được. Kết quả `optical_flow/fa_camera_params.mat`. Không có camera access/hardware activation trong helper. Đọc/kiểm tra kết quả trước dùng; không tự coi calibration đáng tin chỉ vì ít reprojection error.

Xác định hướng cạnh TRÊN ảnh so với thân (`nose`,`tail`,`left`,`right`) và đo khoảng cách camera tới sàn dọc trục quang khi thân ngang ở độ cao thử. R cam->body và offset `[0.10;0;-0.03]` m phải phù hợp lắp thật.

## 3. Stage shadow: kiểm tra trên mặt đất

Disarm, Stop, Save Stabilizer FA. Sau khi có giá trị thật:

```matlab
configure_fa_flight(cameraFile,height_m,imageTop,'shadow',true);
```

`cameraFile`, `height_m`, `imageTop` là dữ liệu đã đo, không phải mặc định. Hàm sao lưu model, đặt stage2, nạp fx/fy/R/h, chạy shared MEX zero-flow check, Update Diagram, lưu và local build. Không download/Start/Arm. Nếu MEX đang được MATLAB nạp và cần build lại sau thay C:

```matlab
clear qdrone_fa_step qdrone_fa_mex
```

Sau đó chạy `optical_flow/build_fa_native.cmd` từ PowerShell và quay lại MATLAB. Trợ lý đã build bản DLL/MEX mới và chạy13 native tests PASS; wrapper/helper mới cần MATLAB kiểm tra tại máy.

Chạy đúng cặp FA bằng quy trình QUARC hiện tại, vẫn Disarm, tháo cánh cho bài thử cảm biến. Gạt SB lên rồi xuống để có request2. Stage2 chỉ tính diagnostic khi Arm=0; đầu ra lệnh góc luôn bằng lệnh tay. Status9 nghĩa là đang tính shadow. Nếu vô tình Arm trong stage2, gate shadow tắt; không chuyển thành điều khiển FA. Chế độ này không phải khóa cứng toàn bộ motor: cơ chế Arm/motor gốc vẫn tồn tại, vì vậy không Arm khi làm bài thử.

Giữ khoảng cách sàn đã đo, nền có họa tiết, đủ ánh sáng. Ghi các đoạn dịch chuyển được quan sát theo thân: tiến/lùi/trái/phải và xoay quanh CG không chủ động tịnh tiến; ghi timestamp bắt đầu/kết thúc mỗi đoạn, mỗi đoạn nhiều hơn0.5s sau khi flow đủ tốt. Stop để đóng log. Không dùng dữ liệu camera đã đứng hình hoặc ảnh không đúng nguồn.

```matlab
verification=verify_fa_shadow_log(logFile,intervals);
```

`intervals` mỗi hàng `[start_s end_s motion]`; motion +1 tiến, -1 lùi, +2 trái, -2 phải, 0 xoay quanh CG. Phải có đủ5loại và ít nhất100mẫu status9 trong từng đoạn. Timestamp lấy theo QUARC log/model time; không bịa các mốc. Helper kiểm tra Disarm toàn log, passthrough đầu ra, stage/R/config không đổi, dấu proxy/góc bù và residual xoay. Lưu `fa_shadow_verification.mat` nếu PASS. Đây là kiểm chứng dấu gần đúng, không phải chứng nhận ổn định bay hoặc đo chính xác vận tốc m/s.

## 4. Tạo bản có tác động điều khiển

Sau khi ground verification PASS với đúng cùng cấu hình, Stop/Save model:

```matlab
configure_fa_flight(cameraFile,height_m,imageTop,'flight',true);
```

Helper từ chối nếu thiếu ground verification hoặc các giá trị fx/fy/h/gain/R không khớp. Stage1 mới cho phép cộng góc bù khi Arm/throttle và các gate khác đủ điều kiện. Mức giới hạn, slew và điều khiển góc/I hiện tại được giữ nguyên. Stage1 local build không tự download/Start/Arm. Thử trên giá phù hợp trước để đánh giá tương tác vòng góc và tải; lệch roll trước đó chưa được coi là đã xử lý. Không coi bước xác minh hướng trên mặt đất là đã xác nhận bay ổn định.

Gạt SB lên/giữa luôn yêu cầu Angle; xuống mới yêu cầu FA. Khi ga/cần ngang/tilt/rate/quality/age không đủ điều kiện, phần bù bằng0. Ga/yaw vẫn nguyên điều khiển hiện tại; không thêm giữ độ cao hoặc heading hold. Mất flow trả về Angle, không tự hạ cánh. Do không có range, không thể tự phát hiện drone đã ra khỏi khoảng độ cao tham chiếu; không tuyên bố mode dùng được ở mọi độ cao.

## Phần chưa thực hiện bởi trợ lý

- Chưa có ảnh/số đo của camera thật để chạy calibration.
- Chưa có log stage2 trên drone để chạy verification.
- Chưa Update Diagram/build hai helper calibration/profile trong MATLAB của người dùng.
- Không download/Start/Arm hoặc bật stage1 trên model hiện tại trong lượt này.

`flow_assist_config.m` vẫn tạo mặc định khóa0. Dùng helper profile cho bản FA đang có; chạy lại toàn bộ setup sẽ tạo lại cấu hình khóa, không tự áp dụng profile đã lưu.
