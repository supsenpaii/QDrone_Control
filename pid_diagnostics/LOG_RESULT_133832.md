# Kết quả log 08/11/2026 13:38:32

Đọc Controller và Motors: 67416 mẫu mỗi file, 500 Hz, thời gian 134.831 s.
QUARC thêm timestamp ở hàng đầu; đã bỏ timestamp này trước khi áp dụng mapping payload.

Đoạn 80–100 s: lệnh roll/pitch bằng 0; throttle 10.5445 N; roll trung bình -2.8945 deg, pitch +0.7502 deg.
Lệnh PWM channels 0..3 trung bình: 0.44839, 0.47424, 0.55612, 0.58375.
Không có clipping bộ trộn hoặc giới hạn mô-men trong đoạn này. Mô-men roll đặt trung bình +0.11855 Nm.

Đoạn 73–75 s: throttle trung bình 8.871 N, roll -2.402 deg.
Đoạn 116–118 s: throttle trung bình 12.899 N, roll -3.465 deg.
Trong log này tăng throttle không làm roll trở về 0.

Gyro roll có lệch nền: đoạn Disarm 40–50 s trung bình -1.32074 deg/s, trong khi góc roll thay đổi rất chậm (-0.00752 deg/s tính theo hai đầu khoảng).
Đoạn 80–100 s gyro roll trung bình -1.34210 deg/s, góc roll thay đổi -0.00221 deg/s.
Đây là dấu hiệu offset gyro; xác nhận bằng phép đo hoàn toàn đứng yên trước khi hiệu chỉnh. Không đủ để giải thích toàn bộ sai số góc -3 deg.

Không thể từ log này kết luận động cơ hỏng hoặc giá treo không tạo mô-men phụ. Chưa đo lực đẩy/RPM từng động cơ, chưa độc lập đo góc thân.
Không thay PID hay tham số điều khiển trong bước phân tích.
