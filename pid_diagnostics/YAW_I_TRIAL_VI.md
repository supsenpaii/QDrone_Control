# Bản thử I cho yaw rate — chưa xác nhận cho bay tự do

Log phiên 16:03:48 cho thấy sai lệch yaw rate kéo dài khi lệnh yaw gần 0. Bộ điều khiển hiện dùng P yaw=0.0395, D yaw=0, không có I yaw. Khi tĩnh, gyro Z lệch khoảng -0.029°/s, nhỏ hơn nhiều so với mức xoay ghi được. Điều này phù hợp thiếu khả năng bù mô-men nhiễu liên tục, nhưng chưa loại trừ sai chiều/thứ tự motor, cánh, cơ khí hoặc actuator không cân bằng.

Đã chuẩn bị bản riêng `Stabilizer_QDrone_QGC_OF_YawI.mdl` bằng script `setup_yaw_integral_trial`. Script lấy model QGC_OF **đã Save** làm nguồn, không thay model đang chạy. Optical flow/telemetry QGC được giữ. Không download, Start hoặc Arm. Công tắc YawI Enable mặc định **0 (tắt)**.

## Trước khi thử điều khiển

Disarm và ngắt nguồn động cơ khi kiểm tra lắp ráp. Kiểm tra cánh CW/CCW đúng vị trí, chiều quay/thứ tự motor đúng mixer của QDrone, cánh không hư/lỏng, motor và khung không lệch. Cần xác minh chiều phản hồi mô-men yaw thực tế trên bố trí thử có kiểm soát phù hợp. Không dùng I để che một mixer hoặc actuator sai dấu; dấu torque tính toán trong log không chứng minh dấu torque vật lý.

Không dùng Position Hold trong giai đoạn này; model chưa có chức năng đó và log flow chưa hợp lệ trong vùng lệnh lực đẩy của phiên vừa phân tích.

## Tạo/kiểm tra/build

Disarm, Stop Stabilizer_QDrone_QGC_OF rồi Save; dừng Mission Server trước khi chuyển model. Trong MATLAB:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'pid_diagnostics'));
setup_yaw_integral_trial(true);
```

Script chạy **test_yaw_i** trên hàm MATLAB thật trước, sau đó tạo model, Update Diagram, kiểm tra gain P/D giữ nguyên và build cục bộ. Cần thấy cả ba PASS: test thuật toán, sơ đồ, local build. Script không dùng normal-simulation compile.

Mặc định I tắt, nên bản mới không được coi là đã sửa hiện tượng xoay. Model mới cần được người dùng kiểm tra/build trong MATLAB; phiên trợ lý chưa chạy được các kiểm tra này.

## Logic thử nghiệm

- Thêm vector `[0;0;I_yaw]` vào torque controller trước khi cộng torque ngoài và limiter hiện có.
- Giữ P/D và bộ trộn hiện có. Cổng Arm lấy tại tín hiệu thực tế đi vào bộ nhân motor của plant.
- Giá trị **khởi đầu thử nghiệm, chưa tune/flight-validate**: Ki yaw=0.01 Nm/rad, cap I=±0.015 Nm, tốc độ tích lũy tối đa 0.003 Nm/s.
- Ki≤0.01, cap≤0.015, slew≤0.003 được kiểm tra trong hàm; cấu hình vượt khoảng thử làm I tắt/reset.
- I reset khi Enable=0, Disarm, throttle<4 N, mode không phải Angle (mode>=0.5), roll/pitch đạt 15°, bất kỳ gyro axis đạt 60°/s, hoặc đầu vào không hữu hạn.
- Khi raw mixer ra ngoài 0..1 hoặc torque limiter cắt, I ngừng tích lũy. Cờ clipping trễ một mẫu 1 ms để không tạo vòng đại số. Có clamp trạng thái I nếu giảm cap.
- Đây là vòng **yaw rate PI**, chưa bổ sung bộ giữ heading tuyệt đối.

## Bật/tắt trong bố trí thử có kiểm soát

Sau khi test/diagram/build đạt và đã xác minh hướng actuator, mở model thử khi đang dừng:

```matlab
m='Stabilizer_QDrone_QGC_OF_YawI';
open_system([m '.mdl']);
open_system([m '/STABILIZER - QDRONE']);
```

`STABILIZER - QDRONE / YawI Enable` là công tắc 0/1. Chỉ đặt 1 cho phép thử phù hợp sau các kiểm tra trên. Có thể đặt qua MATLAB khi model đang dừng:

```matlab
set_param([m '/STABILIZER - QDRONE/YawI Enable'],'Value','1');
save_system(m);
```

Build/download và Start bản thử theo quy trình QUARC đang dùng, chỉ một Stabilizer sử dụng phần cứng. QGC/Python vẫn `--source flight`, TCP18920, schema3; QGC không bật/tắt YawI và các parameter BRG_* không chỉnh gain này. Không coi enable=1 hoặc build đạt là chứng nhận để bay tự do. Nếu chưa có bố trí kiểm chứng tác động yaw phù hợp, giữ enable=0.

Kết thúc phép thử, Disarm và Stop để ghi hoàn tất log. Nếu xoay/dao động tăng, kết thúc phép thử thay vì tăng ga hoặc tăng gain để ép dừng xoay. Có thể tắt I về0 và quay lại model nguồn sau khi dừng bản thử.

## Đánh giá tác dụng

Log riêng `saved_data/Stabilizer_QDrone_QGC_OF_YawI/YawI_*.mat`, biến `yaw_i_data`, 500 Hz theo decimation2. Payload 31 hàng, có thể thêm QUARC timestamp ở đầu:

| Hàng payload | Tín hiệu |
| --- | --- |
| 1 | Thời gian model, s |
| 2–4 | Sai số tốc độ góc, rad/s |
| 5–7 | Tốc độ góc đo, rad/s |
| 8–10 | Góc ước lượng, rad |
| 11–13 | Torque PD gốc, Nm |
| 14–16 | Torque I, Nm; chỉ hàng16 khác0 |
| 17–19 | PD+I trước torque ngoài/limiter, Nm |
| 20–22 | Torque sau limiter, Nm |
| 23 | Arm thực tế vào bộ nhân motor |
| 24 | Throttle command, N |
| 25 | Mode gốc |
| 26 | Enable |
| 27 | Active |
| 28 | Clipping ở mẫu trước |
| 29–31 | Ki, cap, slew |

Cần đối chiếu yaw rate đặt/đo khi lệnh không đổi, I phát triển đúng dấu, sai lệch giảm, không dao động tăng và không chạm cap liên tục. Nếu I chạm cap mà vẫn xoay, chưa đủ cơ sở tự tăng cap; quay lại kiểm tra actuator và mô hình.

Các log Stabilizer/QDrone cũ vẫn có nhưng torque trong log controller giờ có thể gồm I. Không dùng giả định `tau=0.0395*error` của script phân tích model cũ để diễn giải bản YawI; dùng log riêng tách PD/I/Arm.

## Trạng thái xác minh

Người dùng đã chạy test MATLAB, Update Diagram và local build thành công. Log thực tế phiên 16:29:50 xác nhận Enable=1, có I yaw và được cộng vào torque. Tuy nhiên còn yaw tracking error, roll offset và 512 chuyển Active→inactive có I trước đó khác0 trong vùng Arm/ga>=4 N. Xem `YAW_ROLL_RESULT_162950.md`. Bản thử chưa được xác nhận khắc phục yaw hoặc phù hợp cho bay tự do; không tăng cap/loại gate chỉ để ép I tiếp tục tích lũy.
