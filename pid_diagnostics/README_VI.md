# Ghi dữ liệu trước khi tune PID

Model mới: `Stabilizer_QDrone_048378_PID_Diag.mdl` ở thư mục dự án.
Đây là bản sao của file `Stabilizer_QDrone_048378.mdl` đã lưu trên đĩa;
không bao gồm thay đổi chưa Save trong cửa sổ Simulink.

Chỉ thêm Clock, Mux, Scope và To Host File để quan sát. Không thay gain,
bộ trộn, giới hạn, watchdog hoặc tự phát tín hiệu thử. Chưa có autotune online.
Không tự Build, Download, Start hoặc Arm.

## Kiểm tra trong MATLAB R2019a

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'pid_diagnostics'));
check_pid_diagnostics;       % kiểm tra license và Update Diagram
% Nếu Update Diagram thành công, có thể build cục bộ:
check_pid_diagnostics(true); % không download, không Start
```

Đóng model chẩn đoán trước khi chạy script kiểm tra. Script dùng phiên model
riêng rồi đóng nó; không lưu các cập nhật thư viện phát sinh từ Update Diagram.

Đã thấy thư mục `control` trong bản R2019a. Không thấy các thư mục `slcontrol`
và `ident`; đây chỉ là kiểm tra file cài đặt, chưa xác nhận giấy phép.
Script trên in kết quả kiểm tra license và vị trí các hàm tune/nhận dạng.
Lần kiểm tra do người dùng chạy đã trả về license=1 cho cả ba sản phẩm;
`pidtune` và `pidTuner` tìm được, nhưng `tfest` chưa có trên MATLAB path.
License=1 chưa đủ xác nhận các hàm của sản phẩm đã được cài và nạp đầy đủ.

## Thu dữ liệu

1. Kiểm tra giá thử chắc chắn, chỉ trục dự định tune được phép quay. Kiểm tra
   chiều phản hồi và góc IMU trước khi thực hiện thử có lực đẩy.
2. Chạy Mission Server như trước. Dừng model Stabilizer/capture khác đang
   sử dụng cùng phần cứng trước khi chuyển sang model chẩn đoán.
3. Mở model mới, QUARC Build để tải lên drone khi chuẩn bị thử.
4. Giữ Disarm để kiểm tra góc và tín hiệu. Chỉ thử lại trong phạm vi giá thử
   đang cho phép; kết thúc phép thử khi độ nghiêng tăng ngoài phạm vi kiểm soát.
   Không cần kéo ga mạnh để ép drone trở lại ngang.
5. Để xem bốn lệnh động cơ, mở:
   `PHYSICAL PLANT w/ LINEAR THRUST / PID Diagnostic Scope`.
   Thứ tự bốn đường tương ứng PWM channels 0, 1, 2, 3, đơn vị chuẩn hóa 0..1.
6. Giữ nguyên Scope góc `Roll and Pitch Angle Control (deg)` để đối chiếu.
   Scope `STABILIZER - QDRONE / PID Diagnostic Scope` hiển thị ba mô-men điều
   khiển trước giới hạn, đơn vị Nm.
7. Disarm, dừng Stabilizer rồi dừng Mission. Lấy hai file Controller/Motors
   có thời gian tương ứng trong `saved_data/Stabilizer_QDrone_048378_PID_Diag/`.

Hai bộ ghi có Clock riêng theo thời gian model, decimation=2 ở bước cơ sở
0.001 s, dự kiến 500 mẫu/s. Dùng thời gian thực trong log để xác nhận tần số.
Có thêm lưu lượng ghi log; theo dõi computation time/overrun khi kiểm tra.

## Đọc log

```matlab
addpath('pid_diagnostics');
r = analyze_pid_diagnostics( ...
    'saved_data/Stabilizer_QDrone_048378_PID_Diag/Controller_....mat', ...
    'saved_data/Stabilizer_QDrone_048378_PID_Diag/Motors_....mat');
```

Thay đường dẫn bằng tên file thật. Script vẽ góc đặt/đo và bốn lệnh động cơ,
in tỷ lệ thời gian chạm giới hạn 0/1. Thống kê mặc định gồm mọi mẫu Armed:
phải nhìn đồ thị để chọn đúng đoạn throttle gây lệch. Các góc trong file là rad;
đồ thị chuyển sang độ. Chương trình chưa đề xuất hoặc áp dụng gain.

Payload `pid_controller` (22 hàng, chưa tính timestamp QUARC nếu có):

| Hàng | Tín hiệu |
| --- | --- |
| 1 | Thời gian model, s |
| 2..4 | Lệnh roll, pitch (rad), yaw rate (rad/s) sau giới hạn |
| 5..7 | Góc ước lượng trong vector attitude, rad |
| 8..10 | Tốc độ góc đo, rad/s |
| 11..13 | Lệnh tốc độ góc trước giới hạn, rad/s |
| 14..16 | Mô-men điều khiển trước cộng mô-men ngoài/giới hạn, Nm |
| 17..19 | Mô-men sau cộng lệnh ngoài và giới hạn, Nm |
| 20 | Throttle tổng, N |
| 21 | Arm |
| 22 | Flight mode; giữ nguyên quy ước của model gốc |

Payload `pid_motors` (15 hàng):

| Hàng | Tín hiệu |
| --- | --- |
| 1 | Thời gian model, s |
| 2..5 | Bộ trộn trước giới hạn, chuẩn hóa |
| 6..9 | Bốn lệnh động cơ sau giới hạn 0..1, trước cổng Arm |
| 10..13 | Lệnh sau cổng Arm và chuyển đổi pulse offset, trước HIL Write |
| 14 | Điện áp pin, V |
| 15 | Arm |

## Kết quả kiểm tra lúc tạo

Đã kiểm tra cấu trúc MDL, SID không trùng, mọi thuộc tính block và mọi kết nối
cũ được giữ nguyên. Bản sửa sắp xếp mọi block trước đường nối tham chiếu tới nó;
script kiểm tra thêm từng nguồn nối vào Mux, logger và Scope sau khi nạp model.
Kết quả PASS của Update Diagram ở bản cũ không đủ xác nhận các nhánh ghi log:
bản cũ có cảnh báo invalid Destination do thứ tự khai báo, đã được sửa.
Chưa xác nhận lại Update Diagram hoặc compile trong MATLAB
cho bản chẩn đoán này. Các lần mở MATLAB riêng trước đó không hoàn tất khởi
động; công cụ đọc phiên MATLAB sẵn có cũng gặp lỗi nhận diện cửa sổ.
Chạy `check_pid_diagnostics` để xác nhận trước khi sử dụng bản mới.

Gain nhận dạng trên giá thử phụ thuộc quán tính, ma sát và vị trí trục của giá;
không tự coi là gain đã được xác nhận cho bay tự do.
