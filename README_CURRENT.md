# Các model giữ tại thư mục gốc

| Nhóm | File |
| --- | --- |
| Mission gốc | `Mission_Server_QDrone_Manual_Flight_FrSky.mdl` |
| Stabilizer gốc | `Stabilizer_QDrone_048378.mdl` |
| Mission phát triển mới nhất | `Mission_Server_QDrone_Manual_Flight_FrSky_FA.mdl` |
| Stabilizer phát triển mới nhất | `Stabilizer_QDrone_QGC_OF_RYPI_FA.mdl` |
| Độ cao, so sánh lọc và tín hiệu thô | `Read_IMU_Height_Kalman_LP_HP_Raw.slx` |
| Thu thập cảm biến và camera | `QDrone_Capture.slx` |
| Thu thập IMU | `QDrone_Capture_IMU.slx` |

Các bản phát triển được chọn theo nhánh chức năng và thời điểm lưu file.
Ngày 09/11/2026, nhánh telemetry QGC đã được gỡ khỏi Stabilizer FA.
Tên file vẫn giữ `QGC` để tương thích các script cấu hình đang dùng.
FA dùng đường điều khiển FrSky qua Mission Server và các Scope/log Simulink.
Build và Download lại Stabilizer FA để áp dụng thay đổi lên drone.
Thư mục `qgc_bridge` tiếp tục chứa công cụ telemetry cho các model Capture.
Hai bản Capture phục vụ hai phạm vi thu thập riêng. Executable và MEX tương
ứng nằm cạnh model. Các hàm MATLAB hỗ trợ độ cao nằm ở thư mục gốc.

`optical_flow`, `pid_diagnostics`, `qgc_bridge`, `capture_tool` chứa thuật toán,
cấu hình và công cụ. Log dữ liệu nằm trong `saved_data`, `capture_sessions`
và `height_sessions`.

# Các phiên bản lưu trữ

`_archive/20261109_cleanup/` chứa các nhóm sau.

- `models` chứa model trung gian cùng executable/MEX tương ứng.
- `backups` chứa các bản backup, original và autosave.
- `build` chứa mã sinh và các thư mục biên dịch QUARC.
- `history` chứa báo cáo và hướng dẫn các phiên bản trước.
- `legacy_scripts` chứa script thao tác các model đã đưa vào lưu trữ.
- `manifest.json` ghi đường dẫn cũ, đường dẫn lưu trữ và SHA256 cho từng file.

Các script setup trong thư mục công cụ có thể dùng model trung gian làm đầu
vào để tái tạo chuỗi phát triển. Khi chạy lại các script đó, khôi phục model
nguồn được yêu cầu về vị trí cũ trước. Ví dụ, trong PowerShell tại thư mục dự án.

```powershell
& '.\_archive\20261109_cleanup\restore.ps1' -Name 'Stabilizer_QDrone_QGC_OF_RYPI.mdl'
```

Có thể khôi phục nhiều mục bằng danh sách tên.

```powershell
& '.\_archive\20261109_cleanup\restore.ps1' -Name @(
    'Stabilizer_QDrone_048378_RollI.mdl',
    'Stabilizer_QDrone_048378_RollI.rt-linux_x64'
)
```

Chạy script restore với tên mục cụ thể để lấy lại một phiên bản. Nếu đường
dẫn đích đã tồn tại, script dừng để người dùng xử lý xung đột. Thư mục build
của các model đang dùng sẽ được QUARC tạo lại khi Build.

Ngày sắp xếp 09/11/2026.
