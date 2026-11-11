# Độ cao tương đối từ barometer

File model: D:\QDrone\QDrone_Control\Read_IMU_Height.slx
Khối: Ground Reference Height

Đây là ước lượng độ cao tương đối so với vị trí đặt mốc bằng áp suất, KHÔNG phải phép đo khoảng cách trực tiếp xuống mặt đất. Không dùng thay cảm biến range cho optical flow. Thời tiết, gió và luồng khí động cơ có thể gây trôi số đo.

Cách chạy:
1. Dừng Read_IMU và các target khác dùng drone.
2. Trong MATLAB:
   cd('D:\QDrone\QDrone_Control');
   addpath(fullfile(pwd,'capture_tool'));
   open_system('Read_IMU_Height.slx');
   open_system('Read_IMU_Height/Ground Reference Height');
3. QUARC > Build rồi Start. Model chỉ đọc cảm biến; các cờ ghi đầu ra/PWM bị tắt.
4. Đặt drone ở vị trí mặt đất cần lấy mốc, giữ yên ít nhất 20 giây. Chỉ người vận hành xác nhận được vị trí mốc thực tế.
5. Calibration Progress phải lên 1 và Height Valid phải lên 1. Trước đó, NaN là chưa có độ cao hợp lệ.
6. Xem Relative Height m hoặc nhấp đúp Height History để xem đồ thị. Nhấc drone lên chậm rồi hạ về mốc để kiểm tra.

Đầu ra:
- Relative Height m: độ cao đã lọc, hướng lên dương; không chặn số âm do nhiễu/trôi.
- Raw Height m: độ cao chưa lọc.
- Vertical Speed m_s: tốc độ lên/xuống được lọc, chỉ để quan sát.
- Ground Pressure Pa: áp suất mốc p0, không phải khoảng cách mặt đất.
- Calibration Noise m: nhiễu áp suất lúc lấy mốc quy đổi tương đương mét; không phải độ chính xác đã xác minh.
- Status Code: 0 đang lấy mốc, 1 hợp lệ, 2 áp suất sai/không có, 3 đang chuyển động khi lấy mốc, 4 lỗi thời gian, 5 bước nhảy áp suất lớn, 6 nhiễu áp suất quá lớn lúc lấy mốc.

Lấy mốc lại: đổi giá trị khối Rezero toggle 0 to 1 từ 0 sang 1. Lần kế tiếp, đổi 1 -> 0 -> 1. Giữ yên tại mặt đất khi lấy lại mốc.

Cập nhật theo barometer 0,025 s (40 Hz); bộ lọc có hằng số thời gian 0,5 s, vì vậy số hiển thị có độ trễ.
Công thức gần đúng khí quyển chuẩn: h=44330*(1-(p/p0)^(1/5.25588)), p và p0 cùng đơn vị Pa.
Chỉ sử dụng áp suất RAW trước khối Bias Removal. Áp suất đã trừ bias gần 0 không thể dùng trong công thức này.

Xác minh: height_validation.log đã đạt kiểm tra thuật toán, sinh mã C và Update Diagram. Chưa kiểm tra độ chính xác trên phần cứng hoặc dùng để điều khiển bay.

Cập nhật 20 giây: lấy trung bình toàn bộ khoảng hiệu chuẩn và khóa p0 sau khi hợp lệ. Không tự lấy lại mốc khi drone được giữ ở độ cao mới. Mốc chỉ xóa khi bấm reset hoặc khởi động lại model. Thay đổi này cần QUARC Build lại để vào chương trình trên drone; target đang chạy vẫn dùng bản cũ.

