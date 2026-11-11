# Tích phân yaw trong giao diện RYPI

Cập nhật ngày 09/11/2026 theo yêu cầu bỏ I roll. Mã C hiện chỉ tính tích phân
yaw. Đầu ra mô-men tích phân là `[0;0;I_yaw]`, kể cả khi cờ roll cũ được bật.
Các vị trí roll trong state, diagnostic và cấu hình được đặt 0. Giao diện
vector của model/log dùng kích thước lịch sử để các công cụ đọc log tiếp tục
hoạt động. MEX mới có tên `qdrone_yaw_only_mex`.

Model hiện dùng là `Stabilizer_QDrone_QGC_OF_RYPI_FA.mdl`, với các block
`Yaw Integral Controller` và `PD Plus Yaw I`. Cấu hình hiện tại là
`[0;0.01;0;0.015;0;0.003;0;1]`, enable `[0;1]`.
Yaw có Ki 0.01 Nm/rad, cap 0.015 Nm, slew 0.003 Nm/s và Kaw 1/s.

Đóng bản FA đang mở, thêm `pid_diagnostics` vào MATLAB path rồi chạy
`check_yaw_only_fa(true)` để kiểm tra wrapper, Update Diagram và build cục bộ.
Sau khi kiểm tra thành công, mở lại file đã sửa và QUARC Build để tải executable
mới khi chuẩn bị sử dụng. Bản nguồn trước sửa nằm tại
`_archive/20261109_yaw_only/`.

10 test native C và 10000 phép đối chiếu với DLL trước sửa đã đạt. Mô-men,
state và diagnostic của yaw khớp từng mẫu trong phép đối chiếu. Các phần mô tả
roll/yaw bên dưới ghi lại thiết kế trước ngày 09/11/2026.

## Lịch sử thiết kế roll/yaw

Script: `setup_roll_yaw_controller(true)`.

Nguồn: **Stabilizer_QDrone_QGC_OF_YawI.mdl đã Save**. Đầu ra: **Stabilizer_QDrone_QGC_OF_RYPI.mdl**. Thay nhánh I yaw cũ bằng nhánh I roll/yaw, giữ P/D của bản nguồn, vòng góc, optical flow, telemetry QGC, limiter, mixer, watchdog và Arm motor. Không phải Position Hold hoặc giữ heading tuyệt đối. Pitch giữ nguyên, I pitch luôn0.

Chưa tự download, Start hoặc Arm. Mặc định `RYPI Enable Roll Yaw = [0;0]`, nên I mới chưa tác động khi mới tạo model. Bản nguồn và các log gốc không bị sửa.

## Kết quả đọc lại log mới nhất

- `YawI_08-Nov-2026 17-06-48.mat`: chỉ1 mẫu khởi động, không có mẫu Arm/ga>=4 N; không dùng để kết luận về bay.
- Phiên có dữ liệu thử mới nhất: `16:29:50`, 1129.462 s; 564714 mẫu YawI. Log riêng có Arm thật nên chọn Arm>0.5, throttle>=4 N được 641.828 s.
- I yaw đã Enable=1, Active88.61% vùng chọn, ghi512 chuyển Active→inactive khi I trước đó khác0. Điều kiện reset theo mọi ngưỡng động học trong bản cũ làm mất phần bù nhiều lần.
- I yaw chạm cap chỉ1.25% vùng chọn; không có cơ sở chỉ tăng cap để sửa bản cũ.
- Khi roll đặt gần0, roll đo median -2.25°; torque roll tính toán cần bù có median khoảng+0.091 Nm. Không có I roll trong bản cũ.
- Chưa phân biệt lệch tham chiếu IMU/cơ khí/tải tĩnh hoặc xác nhận mọi đoạn là bay tự do. Phải kiểm tra góc thân thật, RC trim, motor/cánh/mixer trước khi thử controller mới có lực đẩy.

Số liệu gốc: `yaw_roll_analysis_162950.json` và `YAW_ROLL_RESULT_162950.md`.

## Thiết kế mới

Giữ nguyên P/D rồi cộng `[I_roll;0;I_yaw]` trước torque ngoài và limiter hiện có.

### Reset, pause và giảm I

| Điều kiện | Tác động lên I |
| --- | --- |
| Disarm, ga<4 N, mode khác Angle, Reset=1, axis Enable=0 | Reset trục tương ứng; trường hợp global thì reset cả hai |
| Dữ liệu/cấu hình không hợp lệ, abs(roll/pitch)>=45° hoặc abs(gyro)>=300°/s | Reset cả hai, status5 |
| abs(roll/pitch)>=15° hoặc bất kỳ abs(gyro)>=60°/s, trong tối đa0.2 s | **Pause tích lũy, giữ mô-men I đã có trong cap**, status2 |
| Điều kiện pause kéo dài>0.2 s | Giảm I về0, tối đa0.03 Nm/s roll và0.015 Nm/s yaw, status4 |
| Trở lại điều kiện bình thường | Tích lũy tiếp với giới hạn slew; timer pause về0 |

Các ngưỡng trên là bảo vệ trạng thái I, **không phải chứng nhận các góc/tốc độ đó an toàn để bay**, và không thay chức năng failsafe của QUARC. Giữ I qua ngắt ngắn là một thay đổi điều khiển cần kiểm chứng, chưa phải kết quả bay đã xác nhận.

### Chống bão hòa theo từng trục

Script đọc mixer4x4 thực tế của model và kiểm tra invertible, motor limiter phải0..1. Tính:

```text
motor_delta = clamp(motor_raw, 0, 1) - motor_raw
force_delta = inverse_mixer * motor_delta
residual_tau = torque_limited - torque_raw + force_delta(2:4)
```

Residual nghĩa là torque đạt được trừ torque yêu cầu trong mô hình allocator. Nó không đo torque thực của motor/ESC. Phản hồi trễ1 tick1 ms để không tạo vòng đại số.

- Nếu sai số đang yêu cầu tích lũy thêm theo hướng không còn khả năng cấp torque, chặn phần tích lũy đó.
- Sai số đổi hướng vẫn được phép unwind.
- Thêm back-calculation `Kaw * residual_tau` để kéo I ra khỏi vùng không thực hiện được.
- Giới hạn slew sau khi cộng phần tích lũy/back-calculation, rồi clamp magnitude I.
- Một trục bị clipping không tự khóa mọi trục khác; dùng residual từng trục.

Tham khảo nguyên lý: [MathWorks anti-windup](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html). Logic được viết bằng C thuần, không phụ thuộc tính năng PID block mới hơn R2019a.

### Cấu hình khởi đầu thử nghiệm

Vector `RYPI Configuration`:

```matlab
[0.02; 0.01; 0.10; 0.015; 0.01; 0.003; 1; 1]
% Ki_roll, Ki_yaw, cap_roll, cap_yaw, slew_roll, slew_yaw, Kaw_roll, Kaw_yaw
```

Ki: Nm/rad; cap: Nm; slew: Nm/s; Kaw:1/s. Cap roll0.10 Nm được chọn để bao phủ mức torque tính toán khoảng0.09 Nm thấy trong log; đây là giả thuyết thiết kế, chưa xác nhận đó là tải nhiễu thật hoặc chọn gain tối ưu. Cap này cao hơn bản roll-I cũ0.03 Nm, nên cần kiểm chứng riêng. Chưa tự tăng P/D. Cấu hình chỉ thay khi model dừng, không điều chỉnh cap trong lúc có lực đẩy.

## Kiểm tra đã thực hiện

1. Biên dịch native C và MEX x64 cho MATLAB R2019a bằng MSVC2017, không có compiler warning trong lần build.
2. **10 test native C đã PASS**: dấu, cap/slew, pitch không đổi, enable riêng, reset, pause không xóa I, pause kéo dài giảm I, directional anti-windup/unwind, ma trận inverse column-major và clipping chéo trục. Có20000 đầu vào ngẫu nhiên và một plant tuyến tính tổng hợp để kiểm tra hội tụ; plant đó không phải mô hình đã nhận dạng của QDrone.
3. Replay564714 mẫu log qua **chính lõi C này**: hữu hạn, không vượt cap, không tích lũy khi Disarm/ga thấp, I pitch=0. Tập đầu vào cũ cho8 sự kiện hard reset yaw về0 thay vì512 chuyển Active→inactive có I của bản cũ; nhiều mẫu pause giữ được I.

**Giới hạn replay:** đây là kiểm tra mở vòng trên trajectory của controller cũ; ghi2 ms khác target1 ms, phản hồi allocator dựng lại từ lệnh cũ. Không dự đoán góc/tốc độ mới, không chứng minh ổn định hoặc giảm yaw/roll thực. Cap có thể chạm nhiều vì error cũ không thay đổi theo I mới. Không dùng kết quả này để tự tăng cap.

Kết quả: `roll_yaw_native_tests.log`, `roll_yaw_replay_latest.json`, `roll_yaw_build_manifest.json`.

## Tạo và build trong MATLAB

Người dùng đã chạy thành công test MATLAB/MEX, Update Diagram, kiểm tra đường nối gốc và local build Linux x64. Lỗi parse ban đầu đã được xử lý bằng cách đăng ký source/header trong lớp `QDroneRYPIBuildable < coder.ExternalDependency`, thay cho tìm đường dẫn trong mã chart. Đã đối chiếu mã C sinh: gọi đúng lõi C với state persistent, I được cộng trước torque limiter, residual được trễ1 tick, Enable=[0;0] và cấu hình đúng. Executable được xác nhận810304 byte tại lần kiểm tra. Vẫn có warning `may be used uninitialized` trong mã sinh sensor/communication giống nhóm đã thấy ở bản nguồn; không coi build thành công là xác nhận các warning vô hại khi chạy. Chưa download/Start/Arm bằng công cụ và chưa có log drone chạy bản RYPI mới để đánh giá yaw/roll thực.

Giữ Disarm, Stop model nguồn rồi Save, dừng Mission Server trước khi chuyển model. Trong MATLAB:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'pid_diagnostics'));
setup_roll_yaw_controller(true);
```

Script sẽ:

- Chạy test wrapper MATLAB qua MEX C đã build; nếu MEX chưa có thì dùng `mex` để build từ cùng nguồn C.
- Tạo bản sao ở staging; xóa nhánh YawI cũ, thêm controller mới và kiểm tra mọi kết nối gốc còn lại.
- Update Diagram, kiểm tra width log59, giữ P/D và Enable=[0;0].
- Lưu model mới, backup nếu đã có, build Linux x64 với DownloadToTarget=off.
- Không tự Start/Arm hoặc gửi lệnh đến drone.

Cần đủ ba PASS: wrapper/MEX, diagram, local build. Nếu có lỗi, chưa dùng model mới; giữ bản nguồn để đối chiếu. Không gọi normal-simulation compile có thể kích hoạt HIL trên PC.

Sau kiểm chứng model, enable riêng từng trục trong phép thử phù hợp: `[1;0]` roll, `[0;1]` yaw, `[1;1]` cả hai. Không bật đồng thời chỉ để thử bay tự do; trước hết cần xác minh reference, chiều motor/cánh/mixer và tác động từng trục. Script setup không tự đổi Enable từ0.

Telemetry QGC vẫn schema3, TCP18920/UDP14550 với `--source flight`; không có mode Position Hold và QGC không chỉnh các gain này. Log controller mới dùng riêng, không diễn giải theo schema YawI cũ.

## Trạng thái hiển thị và log

Display `RYPI Status Roll Yaw` chứa hai mã theo thứ tự roll/yaw:

| Mã | Ý nghĩa |
| --- | --- |
| 0 | Off/reset theo Arm/ga/mode/Enable/Reset |
| 1 | Tích lũy bình thường |
| 2 | Pause ngắn, giữ I đã có |
| 3 | Chống bão hòa/back-calculation |
| 4 | Điều kiện pause kéo dài, đang giảm I |
| 5 | Dữ liệu/cấu hình/envelope nghiêm trọng, đã reset |
| 6 | Clamp tại cap I |

`RollYawPI_*.mat`, biến `roll_yaw_pi_data`, 500 Hz. Payload59 giá trị, QUARC có thể thêm timestamp ở đầu thành60 hàng:

| Hàng payload | Nội dung |
| --- | --- |
| 1 | Clock |
| 2–4 | Sai số rate |
| 5–7 | Gyro |
| 8–10 | Góc ước lượng |
| 11–13 | PD gốc |
| 14–16 | I roll/pitch(0)/yaw |
| 17–19 | PD+I |
| 20–22 | Torque raw sau cộng lệnh ngoài |
| 23–25 | Torque sau limiter |
| 26–28 | Residual allocator ở tick trước |
| 29–30 | Status roll/yaw |
| 31–32 | Thời gian pause roll/yaw |
| 33–34 | Cờ chặn tích lũy theo hướng clipping |
| 35–36 | Delta back-calculation trước slew |
| 37–38 | Cờ clamp cap |
| 39–41 | Arm thực tế, throttle, mode |
| 42–43 | Enable roll/yaw |
| 44–51 | Cấu hình8 giá trị |
| 52–55 | Motor raw trước limiter |
| 56–58 | Lệnh roll/pitch/yaw-rate sau giới hạn |
| 59 | Reset |

Mục tiêu phép thử: giảm sai lệch rate/góc khi command cố định, không gây dao động tăng, không mất I vì ngắt ngắn, không tích lũy theo hướng clipping và reset đúng khi Disarm. Các mục tiêu này chưa được xác nhận bằng log drone chạy controller mới.
