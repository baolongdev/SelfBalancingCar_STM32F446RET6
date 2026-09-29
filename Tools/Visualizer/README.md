# IMU Visualizer

Dashboard Plotly một file để xem dữ liệu UART của xe cân bằng STM32F446RET6.

## Chạy nhanh trên Windows

Mở PowerShell tại thư mục này và chạy:

```powershell
.\run_visualizer.ps1
```

Hoặc double-click `run_visualizer.bat`.

Script sẽ chạy web server tại `http://localhost:8000` và mở:

```text
http://localhost:8000/imu_visualizer.html
```

Sau đó chọn cổng COM, chọn baud `115200` và kết nối UART.

Trang tuning riêng:

```powershell
.\run_balance_tuning.ps1
```

Trang này mở tại `http://localhost:8000/balance_tuning.html` và chỉ quản lý
thuật toán, gain, giới hạn an toàn cùng trạng thái bật/tắt balance.

## Yêu cầu

- Python 3 đã có trong PATH.
- Chrome hoặc Microsoft Edge để sử dụng Web Serial.
- Đóng các ứng dụng khác đang chiếm cổng COM của board.

Plotly và Tailwind đã được tải sẵn trong thư mục `vendor/`, nên dashboard
không cần Internet khi chạy. Chỉ cần mở qua web server localhost để Web Serial
hoạt động.

## Runtime tuning bộ điều khiển

Firmware nhận lệnh trên UART1 và áp dụng cấu hình trực tiếp trong RAM, không
cần build hoặc nạp lại firmware. Mở mục **Balance runtime tuning** trên web:

- **Đọc cấu hình STM32** gửi `BAL GET`.
- **Áp dụng RAM** gửi cấu hình PID, PD hoặc State Feedback.
- **Tắt/Bật balance** gửi `BAL ENABLE 0` hoặc `BAL ENABLE 1`.

Firmware chỉ cho phép `BAL APPLY` khi balance đang tắt. Các giá trị được kiểm
tra giới hạn trước khi áp dụng; khi có fault IMU, stale hoặc quá góc thì tích
phân PID được reset để tránh motor giật khi phục hồi.

Giao thức mẫu:

```text
BAL APPLY alg=PID kp=4 ki=0 kd=0.08 ilim=20 ag=5 rg=0.12 sg=0 pg=0 target=0 max=55 cutoff=30 recover=10 invert=0
BAL GET
BAL ENABLE 0
```

Cấu hình runtime hiện chỉ tồn tại trong RAM. Sau khi reset board, firmware
quay về `BalanceManager::DefaultConfig()`.
