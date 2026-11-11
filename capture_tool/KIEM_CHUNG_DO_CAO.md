# Kiểm chứng độ cao bằng thước: bài A–B–A

Mục tiêu: phân biệt nhiễu, trễ bộ lọc và trôi barometer. Không tự áp hệ số nhân từ một cặp số đo.

1. Cho cảm biến ổn định nhiệt, đặt drone ở vị trí mốc. Lấy mốc đủ 20 giây, đợi Valid = 1.
2. Giữ nguyên tại mốc 10 giây, rồi trong MATLAB đang chạy observer:
   mark_height_truth(0,'ground_before');
3. Nâng cùng một điểm cố định trên drone lên đúng 0,50 m theo thước, giữ nguyên hướng, tránh gió. Giữ yên 10 giây:
   mark_height_truth(0.5,'raised_50cm');
4. Hạ về đúng mốc, giữ yên 10 giây:
   mark_height_truth(0,'ground_after');
5. Nhắn trợ lý “đọc bài 50 cm”. Không bấm lấy lại mốc giữa ba bước.

Ghi chú:
- 0,16 -> 0,27 m là mức tăng 0,11 m. Cần so với mức nâng thật 0,50 m, không so riêng số cuối 0,27.
- File ground_truth.csv ghi timestamp, độ cao theo thước, số thô/đã lọc và áp suất mốc. height.csv chứa diễn biến giữa các mốc.
- Nếu số thô đúng nhưng số lọc phản ứng chậm: kiểm tra lọc và thời gian chờ.
- Nếu quay về nền mà số đo không trở về ban đầu: có trôi/nhiễu hoặc thay đổi điều kiện áp suất.
- Nhiễu hiệu chuẩn tương đương 0,32 m không phải độ chính xác đã chứng nhận; Valid chỉ là phép tính hợp lệ.
- Bộ đo này là độ cao tương đối theo áp suất, chưa phải cảm biến khoảng cách xuống mặt đất.
