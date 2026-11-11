# Filter Comparison có thêm tín hiệu chưa lọc

Mở `Read_IMU_Height_Kalman_LP_HP_Raw.slx`. Bản này được lưu riêng vì
`Read_IMU_Height_Kalman_LP_HP.slx` đang được MATLAB giữ khóa.

`Ground Reference Height/Filter Comparison` có bốn đường, theo thứ tự:

1. `KF + low-pass`: độ cao sau Kalman và low-pass.
2. `Kalman before LP`: độ cao sau Kalman, trước low-pass.
3. `High-pass variation`: biến thiên nhanh ở nhánh high-pass.
4. `Raw height (unfiltered)`: độ cao thô, chưa qua Kalman, low-pass hoặc high-pass.

Raw Height được quy đổi từ áp suất RAW sang mét theo áp suất mốc lấy trong
20 giây. Đường này vẫn dùng mốc hiệu chuẩn để cùng đơn vị và cùng gốc với
các đường độ cao; không vẽ áp suất Pa chung với trục độ cao m.
Ô `Raw Height m` có sẵn vẫn hiển thị giá trị số của cùng tín hiệu chưa lọc.

Dừng model cũ trước khi chuyển sang bản mới dùng chung phần cứng, mở bản
Raw rồi QUARC Build và Start. Lấy mốc khi đứng yên đủ 20 giây như trước.
Các tham số lọc và thuật toán giữ nguyên so với bản LP_HP.

Đã kiểm tra cấu trúc SLX, bốn cổng Scope, chú giải và nhánh dây từ output
số 2 đến ô Raw Height và Scope. Mã thuật toán và cấu hình phần cứng/solver
giữ nguyên. Chưa kiểm chứng MATLAB compilation, QUARC Build hoặc đồ thị
trực tiếp trong môi trường tự động.
