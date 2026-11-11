# Phân tích lệch roll và giả thuyết motor 4 yếu — 08/11/2026

Phân tích chỉ đọc log, không thay gain/model và không gửi lệnh tới drone.

## Dữ liệu và phương pháp

Đọc lại trực tiếp các MAT RYPI hoàn tất từ 17:33:52 đến 17:48:33 bằng `analyze_roll_motor_bias.py`. Phiên 17:39:42 không đạt kiểm tra cấu trúc MAT của trình đọc nên bị loại, không suy diễn kết quả. Đối chiếu thêm báo cáo phiên YawI 16:29:50.

Chọn Arm thực >0.5, throttle command >=4 N, mode<0.5. Tập chậm bổ sung |gyro cả ba trục|<10 độ/s, |roll,pitch ước lượng|<10 độ. Tập roll trung tính bổ sung |roll command|<0.1 độ. Các điều kiện này không chứng minh drone đang bay tự do hoặc đã đạt cân bằng; có thể gồm cất/hạ cánh, giá thử hoặc tác động ngoài.

## Kết quả

- Phiên YawI 16:29:50, roll I=0: roll command gần 0 cho median roll -2.25 độ. Đoạn 710–720 s cần command +3.49 độ để roll đo trung bình -0.10 độ. Có nhu cầu bù tĩnh từ trước bản RYPI.
- Phiên 17:44:25, Enable=[0;1]: sai số roll tuyệt đối trung bình 3.41 độ trong vùng Arm/ga; I roll=0.
- Phiên 17:46:11, Enable=[1;0]: sai số roll tuyệt đối trung bình 1.66 độ; I roll đạt tối đa 0.0762 Nm. Tuy nhiên vùng chọn chỉ 12.766 s, lệnh/pitch/yaw khác phiên kia nên không phải phép so sánh A/B. Trong tập chậm và roll command=0 chỉ có 2.164 s, roll mean -0.086 độ, MAE 0.544 độ; quá ngắn để xác nhận ổn định.
- Phiên 17:47:32, Enable=[1;1]: vùng chọn chỉ 6.356 s, roll command trung bình 16.10 độ, vượt ngưỡng mềm 15 độ của I. Đây không phải đoạn giữ ngang thích hợp để kết luận hai nhánh I xung đột.
- Phiên 17:48:33, Enable=[0;1]: vùng Arm/ga 265.35 s, roll tracking MAE 3.64 độ. Trong tập chậm 133.202 s, roll command mean +2.923 độ nhưng roll đo mean -0.511 độ; PD roll mean +0.1355 Nm. Khi roll command=0 và chuyển động chậm (24.488 s), roll mean -2.701 độ, median -3.508 độ, PD roll mean +0.1086 Nm. I roll luôn 0. Hướng bù roll dương lặp lại.
- Trong tập chậm phiên cuối, torque pitch mean -0.1026 Nm và yaw mean -0.01338 Nm. Lệnh motor raw mean theo kênh model là [0.4032,0.6548,0.6647,0.6415]. Motor 4 không phải kênh có command trung bình lớn nhất; đây là lệnh, không phải RPM/lực đẩy. Thống kê clipping từng kênh lần lượt khoảng [0.098%,0,0,0.096%]. Lệch roll kéo dài không đi kèm bão hòa lệnh thường xuyên ở tập này.

## Ý nghĩa đối với motor 4

Mixer lưu trong model dùng thứ tự [thrust,roll,pitch,yaw]; hàng kênh 4 là [0.0489,+0.4581,-0.5566,+5.1335]. Giả sử đúng thứ tự motor vật lý, cấu hình hình học và mô hình lực lý tưởng, mất lực kênh 4 tạo nhiễu roll âm, pitch dương, yaw âm; bộ điều khiển cần bù ngược dấu. Bù roll dương/pitch âm trong log phù hợp hai thành phần đầu. Nhưng yaw tổng trung bình đang âm, nên mô hình 'chỉ motor 4 giảm đồng đều lực đẩy và torque phản lực' không giải thích toàn bộ ba trục. Không loại trừ motor 4 yếu kèm sai số cánh/drag/motor khác hoặc thay đổi tải; cũng không suy ra phần trăm yếu từ log.

Không có RPM hoặc lực đẩy đo riêng từng motor trong các kênh đang phân tích. Chưa đối chiếu kênh 4 model với số 4 người dùng chỉ trên khung. Vì vậy log chứng minh lệch và nhu cầu bù, chưa xác nhận riêng motor 4 là nguyên nhân duy nhất.

## Hướng khắc phục

1. Nếu đã quan sát motor 4 yếu, xử lý cụm cánh/motor/ESC/đầu nối trước khi tiếp tục tuning bay: xác nhận cánh đúng chiều và không hỏng, trục/bạc đạn không kẹt, kết nối chắc, linh kiện đúng thông số; kiểm tra lực đẩy trên giá đo được bảo vệ. Tháo cánh và ngắt nguồn khi kiểm tra cơ khí/dây. So sánh cùng điện áp và cùng command bằng phép đo lực/RPM; không lấy âm thanh hoặc motor command làm phép đo lực.
2. Đặt thân đúng ngang khi Disarm, kiểm tra roll/pitch estimate và RC command trung tâm. Nếu estimate lệch khi thân thật ngang, sửa tham chiếu/calibration trước; I không tự sửa sai góc tham chiếu. Kiểm tra trọng tâm và thứ tự motor/mixer.
3. Giữ cấu hình yaw hiện có làm mốc. Sau khi phần cứng và tham chiếu đúng, đánh giá lại I roll đã có sẵn trong RYPI bằng thử có kiểm soát, giữ nguyên các gain khác và log đủ command/response, I, residual và motor outputs. Không coi phiên [1;1] cũ là phép thử giữ ngang hợp lệ.
4. Cap roll hiện tại 0.10 Nm thấp hơn mức PD roll trung bình 0.1086–0.1355 Nm trong các tập vừa nêu. Đây chỉ là torque command quan sát, không phải torque nhiễu đo được. Vì vậy không bảo đảm bật I roll sẽ xóa hết lệch; cũng không dùng con số này để tự tăng cap khi motor đang yếu. Sau sửa cơ khí mới xác định lại lượng bù cần thiết.
5. Anti-windup hiện tại dựa trên giới hạn torque và clipping command 0..1, không có phản hồi lực/RPM. Nó không trực tiếp nhận biết motor yếu dưới giới hạn command. Không thêm hệ số boost riêng motor 4 từ các log này: cần đặc tính lực/command được đo và allocator/AW nhất quán nếu về sau có hiệu chỉnh hiệu suất.

Chi tiết số liệu tái lập: `roll_motor_bias_analysis.json`. Các phiên khác điều kiện nên không kết luận ưu thế ổn định chỉ từ các trung bình liên phiên.
