# Yaw còn xoay, roll lệch — phiên 08/11/2026 16:29:50

Đọc log YawI, Stabilizer và đối chiếu timestamp; clock khớp chính xác. Chọn vùng Arm thực tế>0.5 và throttle command>=4 N được 641.828 s dữ liệu, không tự coi mọi mẫu trong vùng này là drone đang bay tự do. Phiên tổng dài 1129.462 s, có nhiều đoạn thử/maneuver. Chưa có xác nhận của người dùng về bay tự do, giá thử hoặc cầm tay.

## Các kết quả đã xác nhận

- YawI Enable=1 toàn phiên, Ki=0.01, cap=0.015 Nm, slew=0.003 Nm/s.
- Active trong 88.61% vùng chọn; cờ clipping trong 2.87%; I chạm cap chỉ 1.25% vùng chọn.
- Log 500 Hz ghi nhận **512 lần Active 1→0 khi I trước đó khác0**, trong vùng Arm và throttle>=4 N. Hàm hiện reset I nếu vượt ngưỡng góc/gyro, nên các lần này làm mất phần bù đã tích lũy. Không tăng Ki/cap để bù cho việc reset.
- Các vi phạm gate có chồng lặp: roll>=15° 3571 mẫu, pitch>=15° 9140 mẫu, |gyro_roll|>=60°/s 7882 mẫu, |gyro_pitch|>=60°/s 21000 mẫu, |gyro_yaw|>=60°/s 2572 mẫu trong tập inactive. Chưa phân biệt chuyển động thật với rung hoặc tác động tay; không tự bỏ gate hoặc nới ngưỡng dựa trên tổng thống kê.
- Khi yaw đặt gần0 (<0.1°/s), yaw đo trung bình +7.38°/s, median +8.97°/s. Đây là tập mẫu chọn, không phải so sánh A/B cùng điều kiện với phiên trước.
- Trong đoạn 710–720 s, I Active liên tục: yaw đặt -0.063°/s, đo -0.447°/s, I trung bình -0.00931 Nm. Điều này cho thấy nhánh I có tác động trong ít nhất một đoạn; không chứng minh ổn định ở mọi điều kiện.
- Khi roll đặt gần0 (<0.1°), roll đo trung bình -1.98°, median -2.25°. Trong đoạn 710–720 s, người điều khiển đặt roll +3.49° để góc đo trung bình gần -0.10°, phù hợp có nhu cầu bù một sai lệch tĩnh. Chưa phân biệt lệch attitude estimate, cơ khí hoặc mô-men nhiễu bằng log này.
- I roll và I pitch bằng0; torque roll total-PD=0 chính xác. Bản YawI không có nhánh I roll. Tác động chéo từ mixer/cơ khí không được loại trừ bởi kiểm tra này.
- Yaw torque total=PD+I khớp sai số dưới7e-18 Nm; PD yaw=0.0395*rate_error khớp chính xác. Như vậy khâu I không chỉ có trong sơ đồ, nó thực sự đã được chạy và cộng vào điều khiển.

## Thứ tự xử lý/tuning

1. Xác minh mặt phẳng thân thật và roll/pitch đo khi Disarm, kiểm tra RC command/trim. Lệch estimate phải xử lý ở cảm biến/khung tham chiếu, không ép PID bù bằng gain. Lệch vật lý cần kiểm tra CG, cánh, motor/ESC và thứ tự/dấu mixer.
2. Tách một đoạn thử có điều kiện rõ ràng, tránh kết luận từ toàn phiên có nhiều lần tác động tay/maneuver. Giữ vòng rate ổn định trước vòng góc. Điều chỉnh một trục/một thông số mỗi lần, có log command/response.
3. Với yaw, giữ nguyên P/Ki/cap/slew hiện tại trong lúc tìm nguyên nhân reset. Logic thử nghiệm reset theo ngưỡng động học đang hạn chế khả năng giữ phần bù. Cần thiết kế lại/kiểm chứng phân biệt reset bắt buộc (Disarm, tắt, ga thấp, lỗi) và pause/hold khi điều kiện tạm thời không cho tích lũy; không đơn giản xóa gate hoặc tăng ngưỡng để tiếp tục bay.
4. Chỉ tăng I khi đã có đoạn Active liên tục, không clipping, sai lệch rate kéo dài và cơ khí/ước lượng đúng. Dữ liệu hiện tại không ủng hộ tăng cap vì chỉ1.25% mẫu chạm cap.
5. Với roll, sau khi xác minh reference và cơ khí, cân nhắc I vòng rate với antiwindup để bù offset tĩnh. Bản roll-I cũ có nguồn AutoCal khác, không tải đè lên bản QGC_OF_YawI hiện tại. Không tăng outer angle P để thay thế việc xử lý bias/tải tĩnh.

Chưa thay model/gain hay download vào drone trong lần phân tích này. Chưa đề xuất bộ gain được coi là flight-validated. Số liệu chi tiết: `yaw_roll_analysis_162950.json`.

Nguyên lý rate-first và sai lệch rate kéo dài của I: [PX4 manual advanced tuning](https://docs.px4.io/v1.16/en/config_mc/pid_tuning_guide_multicopter). Chỉ tham khảo nguyên lý; không áp parameter/gain PX4 trực tiếp vào QUARC.
