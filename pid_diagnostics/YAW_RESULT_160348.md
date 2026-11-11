# Phân tích yaw — phiên 08/11/2026 16:03:48

Đã đọc cặp `Stabilizer_08-Nov-2026 16-03-48.mat`, `QDrone_08-Nov-2026 16-03-48.mat` và OpticalFlow cùng phiên. Dài 175.332 s, 175252 mẫu controller/plant. Log phiên 16:06:51 vẫn 0 byte tại thời điểm kiểm tra nên chưa được phân tích. Không thay gain/model hoặc điều khiển drone.

Đối chiếu Mux trong `Stabilizer_QDrone_QGC_OF.mdl` trước khi chọn hàng dữ liệu. Log controller chuẩn có 20 hàng gồm timestamp, sample/computation time, ba cờ safety, mode và command/torque/throttle; **không có cờ Arm**. Không được coi hàng watchdog là Arm. Các khoảng chọn bằng throttle command >4 N là khoảng có lệnh lực đẩy, không tự chứng minh drone đang bay hoặc motor gate đang Arm. Không có đo RPM/thrust thực.

## Bằng chứng

| Thời gian model | Yaw đặt trung bình | Gyro yaw trung bình | Thay đổi yaw ước lượng | Mô-men yaw trung bình |
| --- | --- | --- | --- | --- |
| 0–25 s, throttle=0 | 0°/s | -0.029°/s | -0.72° | +0.000020 Nm |
| 34–39 s | -0.105°/s | +5.121°/s | +25.59° | -0.00360 Nm |
| 44–48 s | -2.481°/s | +10.819°/s | +43.28° | -0.00917 Nm |
| 73–76 s | 0°/s | +16.846°/s | +50.52° | -0.01161 Nm |

Chọn các mẫu có throttle command >4 N và |yaw command|<0.1°/s được 21.125 s dữ liệu; tốc độ yaw đo trung bình +10.00°/s, median +12.08°/s theo trục QUARC. Đây là dữ liệu gốc, không phải giá trị đổi dấu để hiển thị QGC.

Mô-men yaw trước giới hạn khớp chính xác với:

`tau_z = 0.0395 * (yaw_rate_command - gyro_z)`

Sai số kiểm tra số học lớn nhất 0 trong các mẫu throttle>4 N. Gain D của yaw là 0 và không có nhánh I yaw trong model. Như vậy model có phản hồi tốc độ yaw dạng P; không có bộ giữ heading hoàn chỉnh. Lệnh mô-men có dấu chống lại tốc độ quay, nhưng điều này **chưa xác minh** dấu mô-men vật lý của motor/mixer/cánh.

Mỗi kênh lệnh motor chạm giới hạn 0/1 dưới 0.3% trong vùng chọn. Không có bằng chứng yaw limiter là nút thắt chính ở các cửa sổ trên. Motor0 được ra lệnh thấp hơn các motor khác, nhưng các số đó là lệnh mixer; không thể kết luận motor0 hỏng hoặc lực đẩy từng motor từ log này.

Flow valid toàn phiên 10.33%; khi đồng bộ theo timestamp với throttle command>4 N, valid=0 ở toàn bộ mẫu được chọn. Vì vậy log có du/dv truyền lên QGC không chứng minh flow dùng được lúc thử có lực đẩy. Phải kiểm tra nền/ảnh/ánh sáng/blur/rung và bù xoay, không nới ngưỡng valid để ép phép đo qua.

## Hướng xử lý

1. Dừng thử Position Hold; nhánh flow hiện chỉ quan sát, chưa điều khiển yaw/position.
2. Kiểm tra đúng cánh CW/CCW, chiều quay/thứ tự motor theo mixer của QDrone, độ chắc/cân bằng cơ khí và phản ứng mô-men yaw trên bố trí thử phù hợp. Không dùng lệnh motor trung bình để tự bù lệch cố định.
3. Nếu hướng actuator và cơ khí đúng, bổ sung PI cho vòng yaw rate với chống bão hòa, giới hạn mô-men I và tốc độ tích lũy, reset khi Disarm/ga thấp, lưu trạng thái I/Arm thật để kiểm chứng. P đơn thuần có thể để lại sai lệch tốc độ khi có mô-men nhiễu liên tục; đây là giả thuyết phù hợp log, chưa phải xác nhận nguyên nhân cơ khí.
4. Chưa chọn gain mới từ log hiện tại. Không tự tăng P nhiều lần hoặc thêm offset motor cố định. Cần thử có kiểm soát và đối chiếu yaw rate đặt/đo, mô-men, Arm thật và saturation.

Tham khảo nguyên lý I và sai lệch rate kéo dài: [PX4 rate PID tuning](https://docs.px4.io/v1.16/en/config_mc/pid_tuning_guide_multicopter). Gain PX4 không được sao chép trực tiếp sang model QUARC.

Số liệu máy đọc: `yaw_analysis_160348.json`. Chạy lại phân tích trên đúng phiên bằng `qgc_bridge/.venv/Scripts/python.exe pid_diagnostics/analyze_qgc_yaw.py --session "08-Nov-2026 16-03-48"` từ thư mục dự án.
