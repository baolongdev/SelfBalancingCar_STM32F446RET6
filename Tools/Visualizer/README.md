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

## Yêu cầu

- Python 3 đã có trong PATH.
- Chrome hoặc Microsoft Edge để sử dụng Web Serial.
- Đóng các ứng dụng khác đang chiếm cổng COM của board.

Plotly và Tailwind đã được tải sẵn trong thư mục `vendor/`, nên dashboard
không cần Internet khi chạy. Chỉ cần mở qua web server localhost để Web Serial
hoạt động.
