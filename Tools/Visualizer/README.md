# IMU Visualizer

Dashboard Plotly một file để xem dữ liệu UART của xe cân bằng STM32F446RET6.

## Chạy nhanh

Từ thư mục gốc của project, chạy:

```sh
make visualizer
```

Lệnh này chạy web server tại `http://localhost:8000` và mở:

```text
http://localhost:8000/imu_visualizer.html
```

Sau đó chọn cổng COM, chọn baud `115200` và kết nối UART.

Đổi cổng nếu 8000 đang bận: `make visualizer PORT=8080`.

## Yêu cầu

- Python 3 đã có trong PATH.
- Chrome hoặc Microsoft Edge để sử dụng Web Serial.
- Đóng các ứng dụng khác đang chiếm cổng COM của board.

Plotly và Tailwind đã được tải sẵn trong thư mục `vendor/`, nên dashboard
không cần Internet khi chạy. Chỉ cần mở qua web server localhost để Web Serial
hoạt động.

## Runtime tuning bộ điều khiển

Firmware nhận lệnh trên UART1 và áp dụng cấu hình trực tiếp trong RAM, không
cần build hoặc nạp lại firmware. Gửi lệnh bằng `make uart-send CMD="..."`,
`make uart-get`, `make uart-enable` / `make uart-disable` (dùng
`uart_cli.ps1`), hoặc chạy thử tự động bằng `make autotest`.

Balance luôn bật sau khi boot: xe tự chạy khi góc lệch `<= recover` (mặc định
10°), dừng motor khi lệch `>= cutoff` (mặc định 70°, coi như xe ngã) và tự chạy
lại khi được dựng lên trong `recover`. `BAL APPLY` / `BAL DEFAULT` áp dụng ngay
khi xe đang chạy (tune live); chỉ `BAL SAVE` cần `BAL ENABLE 0` trước vì xóa
flash làm CPU dừng ~1-2 s. Các giá trị được kiểm
tra giới hạn trước khi áp dụng; khi có fault IMU, stale hoặc quá góc thì tích
phân PID được reset để tránh motor giật khi phục hồi.

Giao thức mẫu:

```text
BAL APPLY alg=PID kp=4 ki=0 kd=0.08 ag=5 rg=0.12 sg=0 pg=0 target=0 max=55 cutoff=30 recover=10 invert=0
BAL GET
BAL ENABLE 0
```

Lệnh tune / chẩn đoán thêm:

```text
BAL LOG 8                      # log nhanh [T] mỗi 8 ms (0 = tắt) cho uart_capture.ps1 / tune_analyze.py
BAL APPLY drift=0.02 drift_tau=0.5 trim_rate=0.01 drift_max=3   # chống trôi + tự học điểm cân bằng
BAL YAW kp=0.5 kd=0.05 sign=1  # giữ hướng bằng gyro Z (mặc định tắt); tự tắt nếu lệch > 60°
BAL YAW HOLD                   # lấy hướng hiện tại làm hướng giữ
BAL ODOM RESET                 # đặt lại bản đồ 2D về (0,0), yaw 0
BAL ODOM k=0.008               # m/s cho mỗi 1% PWM (calibrate: chạy thẳng 1 đoạn, đo bằng thước)
```

Visualizer vẽ đường đi 2D ước lượng từ log `[ODO]` (yaw = tích phân gyro Z,
vận tốc = k × PWM). Chưa có encoder nên chỉ dùng để xem xe trôi/xoay về hướng
nào; quãng đường sai số khoảng 20–30%.

```powershell
.\uart_capture.ps1 -Command "BAL APPLY kp=22 kd=0.2" -Seconds 8 -Out run.log
python tune_analyze.py run.log
```

`BAL APPLY` chỉ đổi cấu hình trong RAM. Dùng `BAL SAVE` để lưu vào flash
(sector 7); khi boot firmware load cấu hình đã lưu nếu hợp lệ, ngược lại dùng
`BalanceManager::DefaultConfig()`.
