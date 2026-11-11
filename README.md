# Quanser QDrone Control

Mã nguồn MATLAB/Simulink + QUARC để điều khiển Quanser QDrone qua tay điều khiển FrSky, cùng bộ xử lý optical flow, tích phân yaw, thu cảm biến và cầu nối telemetry QGroundControl.

## Thành phần

| Thành phần | Tệp/thư mục |
| --- | --- |
| Mission Server trên PC Windows | `Mission_Server_QDrone_Manual_Flight_FrSky_FA.mdl` |
| Stabilizer trên QDrone Linux x64 | `Stabilizer_QDrone_QGC_OF_RYPI_FA.mdl` |
| Cặp model gốc để đối chiếu | `Mission_Server_QDrone_Manual_Flight_FrSky.mdl`, `Stabilizer_QDrone_048378.mdl` |
| Thu IMU / camera | `QDrone_Capture_IMU.slx`, `QDrone_Capture.slx`, `capture_tool/` |
| Đo độ cao tương đối | `Read_IMU_Height_Kalman_LP_HP_Raw.slx`, các hàm `qdrone_*height*.m` |
| Optical flow, công tắc SB, Flow Assist | `optical_flow/` (MATLAB + C) |
| Tích phân yaw, anti-windup, gyro calibration | `pid_diagnostics/` (MATLAB + C) |
| Telemetry MAVLink chỉ đọc | `qgc_bridge/` (Python) |

Tên Stabilizer vẫn có `QGC`, nhưng nhánh telemetry QGC đã được gỡ khỏi cặp FA ngày 09/11/2026. Cặp FA dùng Mission Server/FrSky và Scope/log Simulink. QGC không thay thế Mission Server và không gửi lệnh bay qua bridge.

## Yêu cầu

- Máy Windows với MATLAB/Simulink. Môi trường gốc: MATLAB R2019a, QUARC và Visual Studio 2017 x64. Phiên bản khác cần kiểm tra tương thích thực tế.
- MATLAB Coder, Simulink Coder và các toolbox/blockset cần bởi model; Computer Vision Toolbox nếu hiệu chuẩn camera bằng `cameraCalibrator`/`calibrate_fa_camera`.
- QUARC và thư viện/driver QDrone do Quanser cung cấp, target Linux x64 trên drone, tay FrSky đã được Windows nhận diện.
- Python 3.10+ và `qgc_bridge/requirements.txt` nếu dùng telemetry QGC. Phần điều khiển Simulink không cần Python.

Repository chứa mã dự án; MATLAB, QUARC, toolchain, firmware và thư viện thương mại phải được cài riêng bằng giấy phép hợp lệ. Không kèm executable, MEX, cache build hay log đo cũ.

## Thiết lập sau khi tải mã

Giải nén hoặc clone repository vào một thư mục bất kỳ. Trong MATLAB, chuyển **Current Folder** tới thư mục có README này rồi chạy:

```matlab
setup_project;
mex -setup C          % Chọn compiler C tương thích, thường chỉ cần một lần
build_mex;            % Build cả gateway FA cũ/timed, SB và yaw-only từ C
run_offline_tests;    % Chỉ kiểm tra tổng hợp, không kết nối drone
```

`setup_project` chỉ thêm đường dẫn. Không chạy các script `setup_*` để bắt đầu lần đầu: nhiều script tạo lại model hoặc thay cấu hình thử nghiệm. Dùng trực tiếp cặp model FA có sẵn.

Các script `optical_flow/build_fa_native.cmd`, `build_sb_native.cmd`, `pid_diagnostics/build_ry_native.cmd` tạo thêm DLL cho test Python trên Windows. Trong bản xuất `release/QDrone_Control`, các script nhận `MATLAB_ROOT` và `VCVARS64` qua môi trường; mặc định là môi trường gốc nêu trên.

## Kết nối và chạy cặp điều khiển

1. Chuẩn bị QDrone, FrSky và máy PC theo quy trình của phòng thí nghiệm. Khi kiểm tra trên bàn, Disarm và tháo cánh; xác nhận E-stop trước khi thử động cơ.
2. Mạng hiện tại trong model: PC `192.168.2.5`, drone `192.168.2.52`. Kiểm tra **Communication**, địa chỉ Mission Server và URI target trong cấu hình QUARC ở cả hai model; đổi theo mạng thực tế nếu cần. QUARC target manager trên drone hiện dùng `tcpip://192.168.2.52:17000`.
3. Dừng model Capture/Read_IMU hoặc model khác đang chiếm HIL/camera trước khi chạy Stabilizer.
4. Mở hai model:

   ```matlab
   setup_project;
   open_system('Mission_Server_QDrone_Manual_Flight_FrSky_FA.mdl');
   open_system('Stabilizer_QDrone_QGC_OF_RYPI_FA.mdl');
   configure_fa_flight;  % Chỉ đọc và hiển thị stage/thông số hiện tại
   ```

5. Kiểm tra Mission dùng QUARC Windows x64 trên PC; Stabilizer dùng QUARC Linux x64 trên drone. Kiểm tra ánh xạ trục/kênh tay điều khiển và trạng thái Disarm/E-stop.
6. Build hai model bằng QUARC. Build lại từ nguồn trên máy đang dùng, tải đúng executable tới target tương ứng. Chỉ Start theo quy trình QUARC của hệ thống sau khi kiểm tra cấu hình. Bộ mã không tự Start/Arm drone.
7. Theo dõi kết nối, IMU, góc nghiêng, gyro calibration và lỗi/watchdog trên Scope. Chỉ Arm/bay theo quy trình thử nghiệm đã được xác nhận cho thiết bị của bạn. Khi kết thúc, hạ cánh, Disarm rồi Stop cả hai model.

**Cấu hình snapshot:** model FA hiện lưu stage **4** (bounded trial), `RYPI Enable Roll Yaw = [0;1]` (chỉ tích phân yaw), và thông số camera/độ cao tham chiếu của lần thử trên máy gốc. Xuất mã không thay gain hoặc stage này. Đây không phải cấu hình phổ dụng cho drone/camera khác và chưa phải xác nhận bay ổn định. SB lên/giữa yêu cầu Angle; SB xuống yêu cầu FA khi các gate thỏa mãn. Flow Assist không có giữ độ cao hay định vị XY theo mét.

Đọc [hướng dẫn FA](optical_flow/FA_FLIGHT_SETUP_VI.md), [pipeline kiểm chứng](optical_flow/FA_DEPLOYMENT_PIPELINE_VI.md), và mã `setup_fa_timing_trial.m`/`setup_fa_bounded_trial.m` cho các stage timed/bounded. Không dùng bằng chứng ground verification của máy khác; tự thu và kiểm chứng với camera/lắp đặt thực tế.

## Thu cảm biến và QGroundControl

Đây là chế độ riêng, không chạy đồng thời với cặp điều khiển chiếm cùng phần cứng. Mở `QDrone_Capture_IMU.slx`, Build/Start bằng QUARC sau khi dừng model bay. Dùng `capture_tool/open_capture_tool.m` với model camera khi cần ghi dữ liệu; xem [hướng dẫn capture](capture_tool/HUONG_DAN.md).

Với QGC, từ PowerShell ở thư mục repository:

```powershell
python -m venv qgc_bridge/.venv
./qgc_bridge/.venv/Scripts/python.exe -m pip install -r qgc_bridge/requirements.txt
./qgc_bridge/.venv/Scripts/python.exe qgc_bridge/qdrone_to_qgc.py --help
./qgc_bridge/.venv/Scripts/python.exe qgc_bridge/qdrone_to_qgc.py
```

Bridge đọc TCP `192.168.2.52:18910` và gửi MAVLink UDP tới `127.0.0.1:14550`. Chỉ một client dùng luồng telemetry tại một thời điểm; đóng dashboard capture khi bridge dùng cổng này. Xem [hướng dẫn QGC](qgc_bridge/HUONG_DAN.md). Test Python ngoại tuyến:

```powershell
./qgc_bridge/.venv/Scripts/python.exe -m unittest discover -s qgc_bridge -p 'test_*.py' -v
```

## Ghi chú bản quyền

Các model gốc/thư viện Quanser giữ quyền của chủ sở hữu tương ứng; bộ mã chưa bổ sung giấy phép phân phối mới.
