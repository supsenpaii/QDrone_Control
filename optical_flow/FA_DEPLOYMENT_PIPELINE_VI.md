**Pipeline hoàn thiện và đánh giá bay Flow Assist — QDrone**

Tài liệu triển khai theo hiện trạng project ngày 08/11/2026. Đây là kế hoạch kiểm chứng và phát triển, không phải xác nhận drone đã bay FA ổn định. Các mốc định lượng dưới đây là mục tiêu thử nghiệm ban đầu do project đề xuất, không phải tiêu chuẩn chứng nhận hoặc bảo đảm an toàn bay.

**Đích cần hoàn thành**

FA hỗ trợ giảm chuyển động ngang khi thả cần trong một khoảng độ cao, ánh sáng và loại nền đã thử. Đẩy cần ngang hoặc SB lên/giữa sẽ trả quyền điều khiển góc cho người lái. Ga và yaw giữ nguyên bộ điều khiển hiện tại. Không có vòng giữ độ cao, giữ heading mới, tọa độ XY theo mét hay giữ điểm lâu dài trong phạm vi này.

Camera một mắt cung cấp pixel/frame; gyro giúp bù xoay; R cam→body xác định hướng; h_ref đo trước chỉ giúp xấp xỉ phần bù camera lệch CG. Không có range live nên không thể tự biết độ cao thực đã ra khỏi vùng được thử. Nếu yêu cầu chuyển sang Position Hold theo mét và giữ độ cao, phải mở một nhánh công việc riêng cho range/định vị và bộ ước lượng phù hợp. [PX4 mô tả optical flow có khoảng cách hướng xuống để ước lượng vận tốc](https://docs.px4.io/v1.18/en/sensor/optical_flow).

Luồng hiện tại:

```text
Camera device 2, ảnh xám 640x480, khoảng 30 Hz
    -> flow du,dv,quality,valid,dt,age,textured,inliers
Gyro 1 kHz -> tích phân cửa sổ frame
Calibration fx/fy + R + offset [0.10;0;-0.03] m + h_ref
    -> bù xoay và lever-arm gần đúng
    -> proxy chuyển động ngang/độ cao (1/s), lọc
    -> góc bù roll/pitch giới hạn
    -> cộng vào lệnh góc khi pilot neutral và gate hợp lệ
    -> limiter gốc -> vòng góc/rate -> RYPI/AW -> mixer -> Arm gate -> motors

SB Rz /9: -1 lên / 0 giữa -> mã0 Angle; +1 xuống -> mã2 yêu cầu FA
Mã2 -> giải mã base Angle và FA request; KHÔNG đưa mã2 vào switch Torque cũ
```

**Hiện trạng và nút thắt đã có bằng chứng**

| Hạng mục | Bằng chứng hiện có | Chưa được xác nhận |
|---|---|---|
| SB | Người dùng xác nhận Rz /9, lên=-1, giữa=0, xuống=+1; code/MEX có 9 native tests | Cần lưu log chuyển nấc trên bản target thực tế |
| Bộ FA | C/MEX có 13 native tests, gồm shadow không thay lệnh | Các test tổng hợp không chứng minh ổn định drone |
| Camera flow | Phiên 19:40:52: 14.498 mẫu, valid 6,50%, 10 giây cuối 0% valid | Chưa xác nhận nguồn gốc chất lượng kém hoặc tín hiệu live hiện tại |
| Calibration | Có helper và Camera Calibrator trong R2019a | Chưa có cameraParams thật trong project |
| Mounting | Offset người dùng cung cấp: +10cm X, -3cm Z, Y giả sử0 | Hướng cạnh trên ảnh và sai số đo offset cần kiểm chứng |
| Roll/yaw | Log cũ có lệch roll và nhu cầu bù; [0;1] được người dùng báo ổn nhất | Không xác nhận riêng motor4 yếu hoặc đã sửa cơ khí |
| Runtime | Đã có các executable FA; từng xử lý lỗi download khi Mission còn chạy | Binary cũ không tự cập nhật khi chỉ sửa C/MEX; chưa có fingerprint live |

Nguồn nội bộ: `flow_signal_194052.json`, `pid_diagnostics/roll_motor_bias_analysis.json`, các manifest native và log build. Log đang0 byte/chưa đóng không dùng để kết luận camera chết hoặc không có dữ liệu.

**Các trạng thái phải phân biệt**

| Tầng | Giá trị | Ý nghĩa |
|---|---|---|
| SB / request | mode0 / mode2 | Người lái chưa/yêu cầu FA |
| Config(1) | 0 | Khóa, không có góc bù |
| Config(1) | 2 | Shadow khi Disarm: tính diagnostic, không cộng góc bù |
| Config(1) | 1 | Cho phép góc bù nếu các gate hợp lệ |
| FA status | 1 | Config khóa/sai |
| FA status | 2 | Gate không đủ hoặc ưu tiên tay |
| FA status | 3 | Chờ flow tốt đủ thời gian |
| FA status | 4 | Flow thiếu/sai/già |
| FA status | 5 | Đang cộng góc bù thực |
| FA status | 9 | Đang tính shadow, không tác động |
| FA status | 8 | Clock/gyro/input không hợp lệ |

Không dùng riêng mode2, tuổi phần mềm hoặc frame_dt≈33ms để kết luận hệ thống sẵn sàng bay.

**P0 — Chốt phiên bản và cấu hình thử**

Việc làm:

- Lưu cặp model thực sự đang dùng, config FA, RYPI, SB và thông số camera; đặt session ID cho từng bài thử.
- Lưu backup và hash C/MATLAB/MDL/executable cùng thời điểm build. Kiểm tra bản shadow mới đã được codegen vào executable, không chỉ build MEX trên PC.
- Xác nhận Mission chạy Windows trên PC; Stabilizer chạy Linux x64 trên 192.168.2.52; không chạy hai model tranh HIL/camera.
- Khi download báo privileges/file lock, kiểm tra model còn nạp và dừng bằng QUARC sau khi hạ cánh/Disarm. Không mặc định lỗi cần quyền Administrator.
- Ghi rõ mốc0 = locked,2 = shadow,1 = flight; giữ khóa trong khi chưa xong P1–P5.

Lệnh đọc cấu hình/trạng thái:

```matlab
cd('D:\QDrone\QDrone_Control');
addpath(fullfile(pwd,'optical_flow'),fullfile(pwd,'pid_diagnostics'),fullfile(pwd,'qgc_bridge'));
configure_fa_flight;
[models,err]=qc_get_loaded_models('tcpip://192.168.2.52:17000');
disp(models);disp(err);
```

Đầu ra: manifest phiên thử và backup cặp model. Chuyển bước khi biết rõ executable/profile nào đang chạy. **Còn cần bổ sung build ID có thể đọc từ target**, vì tên model giống nhau không chứng minh cùng code.

**P1 — Khôi phục camera và chất lượng flow**

Ưu tiên hiện tại: làm khi Disarm, không dùng thêm controller để che dữ liệu cảm biến hỏng.

1. Xem ảnh gốc: đúng camera nhìn xuống, không bị che, ảnh mới khi di chuyển vật trong vùng nhìn. Không chỉ xem ảnh overlay hoặc thời gian Scope tăng.
2. Ghi nguồn device2, kích thước ảnh640x480, sample_time0.033s, độ nét, ánh sáng và exposure nếu có thể đọc. Xác minh không có model khác đang mở camera.
3. Thử nền nhiều họa tiết không phản chiếu; thử đứng yên, dịch chậm X/Y, đổi khoảng cách có đo và xoay. Mỗi lần chỉ đổi một yếu tố để tìm nguyên nhân.
4. Đối chiếu valid,quality,textured,inliers,du/dv,dt,age. Valid=0 không phải vận tốc0. Sentinel age≈1e9 lúc chưa từng có flow hợp lệ phải tách khỏi tuổi flow sau khởi tạo.
5. Kiểm tra giới hạn tìm kiếm: code hiện downsample4 lần và tìm±6pixel ở ảnh nhỏ, nên dịch ảnh gần±24pixel/frame dễ hết miền tìm kiếm. Không tăng tốc bay nếu đang chạm biên; xem xét tốc độ frame/độ cao/thuật toán sau khi đo.
6. Kiểm tra console target: camera errors, watchdog, overrun và thời gian vòng1ms khi camera xử lý. Có data đều không chứng minh CPU đủ deadline.

Mục tiêu đề xuất: trong bài thử nền và ánh sáng dự kiến, ≥95% frame có valid=1 **và** quality đạt gate FA trong các đoạn đứng yên/dịch chậm đủ dài; không có mất flow kéo dài quá age limit0.10s ở điều kiện danh định. Ghi phân bố và chuỗi mất liên tục, không chỉ trung bình. Mất flow chủ động phải được phát hiện và ghi, không tính vào mục tiêu danh định.

Nếu chưa đạt: dừng phát triển gain; xử lý ảnh/driver/exposure/texture/độ nét trước. Chưa có đủ bằng chứng để biết vấn đề hiện tại thuộc phần nào.

Đầu ra: log flow/ảnh mẫu đã đóng, báo cáo valid+quality+gap và console. **Còn cần cơ chế lưu snapshot ảnh đúng camera, frame counter/hardware timestamp nếu driver hỗ trợ, và báo cáo tự động camera freshness/latency.** Các helper calibration hiện đọc ảnh đã có, chưa tự thu ảnh live.

**P2 — Ổn định vòng góc/rate và xác minh cơ khí**

Chạy độc lập với P1 khi có điều kiện; cả hai phải xong trước thử tác động FA.

- Đặt thân thật ngang, so roll/pitch đo và RC command trung tâm. Phân biệt lỗi tham chiếu, trim RC, trọng tâm, cánh/motor/ESC và nhiễu cảm biến.
- Nếu nghi motor4 yếu, đo lực/RPM trên giá phù hợp cùng command/điện áp; motor command trong log không chứng minh lực đẩy. Không tự boost motor4 từ thống kê command.
- Chốt một cấu hình Angle có log bám góc/rate, giới hạn motor/torque và I/AW. Giữ yaw-only [0;1] làm mốc nếu đó là cấu hình thực đã chọn; không tự đổi cả gain và nhánh I cùng lúc.
- Tune vòng rate trước vòng góc, rồi mới vòng hỗ trợ flow. Tham khảo nguyên tắc [PX4 PID tuning](https://docs.px4.io/v1.16/en/config_mc/pid_tuning_guide_multicopter); không chép gain PX4 sang QDrone.
- Chú ý gate FA yêu cầu cả hai lệnh ngang nằm trong±1độ. Nếu cần trim roll≈+3độ như log cũ để giữ ngang, FA sẽ không tham gia. Phải giải quyết bias/vòng trong; không đơn giản mở rộng deadband để ép active.

Chuyển bước khi đã có đoạn bám góc với điều kiện vật lý rõ ràng, không dao động tăng dần hoặc thiếu authority thường xuyên. Ghi sai số steady-state/MAE, rate và saturation; giới hạn chấp nhận phải chốt theo sai số đo và vùng thử trước khi bay, không đặt sau khi xem kết quả để làm PASS.

Đầu ra: baseline Angle, cấu hình cố định và báo cáo log. Nếu chỉ có dữ liệu giá thử/cầm tay, phải ghi rõ; không gọi đó là hover tự do.

**P3 — Calibration và hình học camera/IMU**

1. Thu20–30 ảnh checkerboard đa dạng, ít nhất15 ảnh dùng được, đúng camera flow và đúng640x480. Bảng phải phẳng, rõ; đo cạnh ô thật.
2. Chạy helper hoặc Camera Calibrator; xem độ phủ toàn ảnh, reprojection theo từng view và sai số hệ thống, không chỉ số trung bình. [MathWorks: đánh giá calibration](https://www.mathworks.com/help/vision/ug/evaluating-the-accuracy-of-single-camera-calibration.html).
3. Xác minh fx/fy,principal point,distortion,ImageSize. Helper hiện lấy fx/fy cho trường flow gần tâm; core chưa hiệu chỉnh méo đầy đủ. Nếu méo gây sai số đáng kể trong vùng các patch, cần bổ sung rectification hoặc mô hình trường xoay đầy đủ rồi kiểm tra CPU lại.
4. Xác định cạnh trên ảnh hướng nose/tail/left/right bằng dịch chuyển và dấu ảnh thật. Helpers chỉ hỗ trợ camera nhìn xuống gần vuông góc và bốn hướng mounting này; camera nghiêng khác cần đo extrinsics đầy đủ, không chọn hướng gần đúng tùy tiện.
5. Đo offset thật và độ cao quang học h_ref; không dùng khí áp như range xuống sàn. Dữ liệu hiện là `[0.10;0;-0.03]`m theo body FLU.
6. Đo/ước lượng độ trễ ảnh–gyro từ bài xoay không tịnh tiến. Timestamp phần mềm hiện không phải thời điểm exposure. Nếu residual phụ thuộc tốc độ quay hoặc chiều quay, sửa đồng bộ/bù xoay trước gain.

Lệnh offline (các biến phải chứa dữ liệu đo thật):

```matlab
cameraParams=calibrate_fa_camera(imageFolder,squareSize_mm);
cameraFile=fullfile(pwd,'optical_flow','fa_camera_params.mat');
```

Đầu ra: cameraParams, ảnh nguồn, kết quả fit, R, offset, h_ref và ghi nhận uncertainty. Chuyển bước khi các đại lượng có nguồn đo, hướng đúng và residual xoay đã được đánh giá; không tự coi có file MAT là calibration đã tốt.

**P4 — Hoàn thiện logic mode, gate và khả năng quan sát**

Đã có trong code: SB startup phải gạt lên, giữ lên≥0.15s; xuống≥0.12s mới yêu cầu FA; mất dữ liệu tay cầm≥0.25s mất yêu cầu và cần gạt lên lại. Pilot ngang ra deadband, mất flow, tilt/rate ngoài gate đều bỏ góc bù. Vào FA có slew; thoát hiện trả góc bù về0 ngay, có thể có bước tối đa bằng cap3độ.

Cần kiểm chứng và bổ sung trước khi gọi là hoàn thiện:

- Hiển thị đồng thời `FA requested`, stage, status thực, lý do không active, góc bù và build/profile ID. QGC hiện chỉ telemetry, chưa có nhãn FA riêng; không lấy nhãn QGC làm bằng chứng active.
- Log frame freshness, gyro/ảnh timing và trạng thái chuyển mode; SB hiện có display nhưng chưa có log giao dịch mode chuyên biệt đầy đủ. Cần lưu SBraw,RCfresh,encodedMode,baseMode,request,status cùng clock.
- Chốt chính sách enable/tắt/mất flow/reacquire/pilot override. Không giữ bù cũ khi dữ liệu mất tin cậy; kiểm tra cú nhảy khi bỏ bù ngay có chấp nhận được với vòng góc không. Nếu thiết kế đường thoát mượt mới, phải có giới hạn thời gian và kiểm tra lại mất dữ liệu, không chỉ làm mượt tùy ý.
- Xem có cần tách gate tilt/rate theo từng trục. Bản hiện tại vượt40độ/s ở một trục sẽ bỏ Assist; việc quay yaw có thể làm Assist tắt, đây là hành vi hiện tại cần công bố/kiểm chứng.
- Kiểm tra command/torque/mixer headroom khi FA tác động. Vòng Assist hiện chỉ P+lọc nên không windup riêng; RYPI vẫn cần AW. Nếu sau này thêm I ở vòng ngoài, phải có feedback saturation/pilot-override/reset phù hợp.

Đầu ra: bảng chuyển trạng thái, test trace và log rõ lý do vào/ra. Không sửa bỏ gate chỉ để status luôn5.

**P5 — Shadow trên mặt đất, xác minh đúng dấu**

Sau khi P1/P3 đạt, hạ cánh/Disarm, Stop và Save model. Clear MEX trước rebuild nếu đã nạp bản cũ. `build_fa_native.cmd`/`build_sb_native.cmd` dành cho PowerShell, không gõ như lệnh MATLAB. Không cần rebuild lặp lại khi code không đổi.

```matlab
configure_fa_flight(cameraFile,height_m,imageTop,'shadow',true);
```

Local build không tự download/Start. Khi nạp bản shadow vào target, giữ Disarm và dùng quy trình hiện có; shadow không phải khóa cứng toàn bộ motor. Làm bài cảm biến với cánh đã tháo. Đặt khoảng cách đã đo trên nền phù hợp; SB lên rồi xuống, neutral pilot command.

Ghi các đoạn tiến/lùi/trái/phải và xoay quanh CG, có timestamp bắt đầu/kết thúc thật. Sau Stop đóng log:

```matlab
verification=verify_fa_shadow_log(logFile,intervals);
```

`intervals` mỗi hàng `[start end motion]`, motion=+1 tiến,-1 lùi,+2 trái,-2 phải,0 xoay. Có đủ5loại, mỗi đoạn≥0.5s và≥100mẫu status9. Helper yêu cầu Disarm toàn log, config/R không đổi, đầu ra đúng bằng command tay; kiểm tra dấu proxy và góc bù, residual xoay. Ngưỡng residual0.08/s và proxy0.03/s trong helper là tiêu chí thử thô, không phải bảo đảm chính xác/stability.

Dấu cần thấy theo FLU: chuyển động tiến -> pitch bù âm; lùi -> pitch bù dương; sang trái -> roll bù dương; sang phải -> roll bù âm. Xoay quanh CG không được sinh ra phần bù tịnh tiến đáng kể kéo dài. Đừng diễn giải xoay quanh camera là xoay quanh CG vì lever-arm khác.

Đầu ra: `fa_shadow_verification.mat` đúng profile và trace. PASS chỉ chứng minh tính nhất quán/dấu trong các đoạn đã đánh dấu, không chứng minh đáp ứng kín trên máy bay.

**P6 — Test lỗi, replay và ngân sách runtime**

Trong shadow/bench, kiểm tra:

| Tình huống | Kết quả mong đợi |
|---|---|
| Khởi chạy khi SB xuống | Chưa request FA, cần gạt lên trước |
| SB lên/giữa | Request0, không còn góc bù tác động |
| Pilot ngang ra deadband | Ưu tiên command tay; không tích lũy bù |
| Flow invalid/chất lượng thấp/đứng hình | Không tích lũy; hết thời gian dữ liệu thì bỏ trợ giúp |
| Flow hồi phục | Chờ dữ liệu tốt liên tục theo config, không chèn bù tức thì |
| Mất RC fresh | SB mode vềAngle; cơ chế mất RC gốc phải vẫn hoạt động |
| Clock lùi/gap, NaN hoặc timing mismatch | Gate/reset theo code; không giữ góc bù cũ |
| Arm khi shadow | Shadow gate tắt, không tự chuyển sang stageflight |
| Giới hạn motor/torque | Không vượt limiter; ghi residual/AW và headroom |

Native tests hiện có kiểm tra nhiều trường hợp trên bằng dữ liệu tổng hợp. **Cần thêm test tích hợp MATLAB/codegen và fault injection trên target khi không có tác động cánh**, đặc biệt frame cũ lặp lại nhưng timestamp phần mềm tăng. Không chủ động cắt RC/camera khi đang bay để làm test đầu tiên.

Đo CPU/loop-time, frame rate thực, latency/jitter và watchdog dưới tải camera+logger+telemetry. Nếu nâng cấp thuật toán ảnh/tăng log, kiểm tra ngân sách runtime lại. Tắt bớt display không đồng nghĩa đã khắc phục deadline trên target.

Đầu ra: fault matrix PASS/FAIL, version của test và trace target. Có lỗi chưa giải thích thì quay về phase liên quan.

**P7 — Bản điều khiển và thử có tác động trên giá phù hợp**

Chỉ sau P2–P6, tạo bản stage1 từ cùng profile:

```matlab
configure_fa_flight(cameraFile,height_m,imageTop,'flight',true);
```

Helper yêu cầu ground verification khớp fx/fy/R/h/gain; không bypass bằng cách sửa flag trong MAT. Local build vẫn chưa download/Start. Không dựng lại toàn bộ `setup_flow_assist` vì sẽ đưa config về khóa và có thể ghi đè thay đổi đã đo.

Kiểm tra trên giá thử được thiết kế cho hệ propulsion này: đúng dấu actuator/feedback, giới hạn góc bù, manual override, chuyển SB, camera thấy sàn thật, và điều kiện motor/load. Không cầm drone bằng tay khi motor chạy để thay cho giá thử. Giá cố định hạn chế chuyển động không thể chứng minh hãm ngang trong bay tự do.

Giữ nguyên gain ban đầu trong lần kiểm tra đầu; hiện cap3độ, slew4độ/s, gain0.03s, filter0.1s là cấu hình thử, chưa phải gain đã xác nhận bay. Ghi actualArm, command trước/sauFA, angle/rate,I,torque/motor,quality/age và status. Kết quả phải chỉ rõ giới hạn mô phỏng/giá thử.

**P8 — Thử bay tăng dần trong điều kiện đã chốt**

Thử bởi người điều khiển có khả năng hồi phục bằng Angle, trong khu vực thử phù hợp. Chốt trước vùng bay, mặt nền/ánh sáng, độ cao tham chiếu, giới hạn góc/trôi/tốc độ, tín hiệu kết thúc và cách hồi phục; không chọn một độ cao tùy ý từ ví dụ code. Xử lý vấn đề roll và motor nghi yếu trước.

Trình tự mỗi phiên:

1. Kiểm tra đúng cặp FA, build/profile, flow live và SB. SB lên khi khởi chạy; cất cánh bằng Angle theo quy trình đã được kiểm chứng cho QDrone.
2. Lấy baseline Angle ở cùng điều kiện. Khi ổn định và flow đủ tốt, yêu cầu FA trong khoảng ngắn đã định trước; không đồng thời thêm thao tác yaw lớn hoặc đổi ga.
3. Trả SB lên, đánh giá log. Chỉ kéo dài thời gian hoặc thêm một thao tác khi phiên trước đạt tiêu chí và không có vấn đề chưa giải thích.
4. Mở rộng lần lượt: hãm theoX, hãm theoY, pilot override, chuyển SB, thay đổi yaw nhỏ, rồi các mức độ cao/nền/ánh sáng có đo. Không suy rằng qua một nền là dùng được mọi nền.
5. Khi dao động tăng, phản ứng sai chiều, drift xấu đi, angle/motor chạm giới hạn thường xuyên, flow mất hoặc trạng thái không đúng dự kiến, kết thúc bài thử bằng phương án hồi phục đã chốt. Không tune giữa một tình huống đang mất ổn định.

Không đặt mục tiêu đứng im tuyệt đối. Cần nguồn tham chiếu độc lập (video ngoài với thước/mốc đo, mocap nếu có) để đánh giá trôi/quãng hãm; không dùng chính proxy flow làm cả phép đo đầu vào lẫn bằng chứng giữ vị trí.

**P9 — Tuning có kiểm soát và chốt phạm vi dùng**

So sánh các đoạn cùng điều kiện; đừng lấy toàn bộ hai phiên có command khác nhau làm A/B. Mỗi lần thay một yếu tố và lưu phiên bản:

- Tăng/giảm vòng trong chỉ khi lỗi angle/rate thể hiện ở baseline; đánh giá lại FA sau thay đổi.
- Nếu flow đúng và không trễ lớn nhưng hãm thiếu: đánh giá gain FA trong giới hạn headroom. Nếu đã cap lâu, xác minh authority và nền tảng trước khi tăng cap.
- Nếu lắc qua lại: kiểm tra dấu, delay, chất lượng, rate/angle tracking, rồi mới thay gain/filter. Lọc mạnh thêm có thể làm trễ hơn.
- Nếu chỉ thay yaw là trôi: kiểm tra bù xoay, extrinsics, timestamp, lever-arm và gate40độ/s.
- Nếu thay độ cao thì hiệu quả khác: đây là giới hạn dự kiến khi không range; thu hẹp phạm vi hoặc bổ sung range, không lấy một gain lớn để ép mọi độ cao.
- Không thêm I vòng ngoài để che lỗi đo hoặc motor. Mọi thay đổi gain/R/h cần shadow verification khớp lại theo helper hiện tại.

Báo cáo cần có: valid/eligible fraction và gap; thời gian actual status5; lý do inactive; angle/rate tracking; góc bù/cap/slew; motor/torque saturation và AW; độ trôi và quãng/thời gian hãm đo độc lập; số lần override/hồi phục; độ cao/nền/ánh sáng và battery. Ghi cả bài thất bại, không chỉ chọn đoạn đẹp.

**P10 — Đóng gói, rollback và điều kiện hoàn tất**

Gói bàn giao: cặp MDL/executable cùng build ID, profile camera/R/offset/h/gains, ảnh calibration, shadow evidence, rawlogs và báo cáo so sánh, fault matrix, phạm vi độ cao/nền/ánh sáng đã thử, hướng dẫn SB và giới hạn ga/yaw. Không ghi “ổn định” cho phần chưa được đo.

Rollback phải theo cặp, sau hạ cánh/Disarm: SB lên, Stop đúng model, nạp cặp baseline đã lưu. Không ghép Mission FA phát mode2 với Stabilizer cũ vốn hiểu mode>0.5 làTorque. Không chạy model thứ hai để tranh camera/HIL với model đang chạy. Snapshot source/executable/profile cũ trước cập nhật để tránh mất bản đã thử.

Mode được coi là hoàn tất **trong phạm vi thử đã định** khi: flow đủ tin cậy; calibration/hình học/timing được kiểm chứng; vòng trong bám tốt; shadow+fault+target-runtime qua; chuyển mode/pilot override đúng; các bài bay lặp lại đo độc lập cho thấy giảm trôi/hãm tốt hơn baseline và không tạo dao động/thiếu authority; phiên bản có thể khôi phục. Chưa có các bằng chứng này thì chỉ gọi là bản thử nghiệm.

**Danh sách triển khai còn thiếu trong phần mềm, không nhầm với file đã có**

| Việc | Đã có | Cần bổ sung/kiểm chứng |
|---|---|---|
| Calibration offline | `calibrate_fa_camera.m` | Thu ảnh camera thật, kiểm chứng calibration; rectification nếu cần |
| Nạp profile/stage | `configure_fa_flight.m` | MATLAB Update Diagram/build/runtime của helper mới |
| Chọn SB | `setup_sb_flow_assist.m`, C/MEX | Trace target và RC loss, xác nhận bản đang nạp |
| Controller + shadow | `qdrone_fa_core.c`, C/MEX | Kiểm chứng trên target và mặt đất |
| Ground verification | `verify_fa_shadow_log.m` | Log thật và khoảng chuyển động được quan sát |
| Thu ảnh và timing | Chưa có pipeline hoàn chỉnh | Snapshot/frame counter/exposure-time hoặc latency measurement |
| Báo cáo flow | Có báo cáo phiên19:40:52 | Bộ phân tích tự động nhiều phiên, eligible gaps/latency |
| Visibility/version | Display và FA log | Build/profile ID live, lý do inactive/SB log, nhãn QGC nếu muốn |
| Kiểm thử tích hợp | Native tests | MATLAB+QUARC fault/runtime tests |
| Đánh giá bay | Chưa có bằng chứng FA active bay | Video/mốc đo độc lập, báo cáo A/B phù hợp, phạm vi áp dụng |

**Bước tiếp theo cụ thể:** P0 và P1. Chốt đúng binary, quan sát ảnh mới và tìm nguyên nhân valid6.5%. Không mở stageflight trước để thử xem có tự hết lỗi hay không.
