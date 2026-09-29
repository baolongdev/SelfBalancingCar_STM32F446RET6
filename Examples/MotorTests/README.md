# Bộ mẫu kiểm tra Motor

Thư mục này chứa các đoạn code mẫu kiểm tra driver motor cho xe cân bằng.
Các file không tham gia trực tiếp vào build firmware.

## Danh sách file

- `manual_drive_example.c`
  - Test cơ bản: tiến/lùi/rẽ/phanh theo phase.
- `non_blocking_profile_example.c`
  - Ramp up -> run -> brake hold -> stop (non-blocking).
- `safety_and_fault_example.c`
  - E-stop, watchdog và fault flags.
- `feedback_hooks_example.c`
  - Mẫu cập nhật battery/speed/current feedback, closed-loop, stall detect.

## Cách dùng

1. Kiểm tra xe đã kê bánh khỏi mặt đất.
2. Mở file mẫu cần dùng.
3. Copy các vùng tương ứng vào `Core/Src/main.c` hoặc dùng API tương đương trong `app_main.cpp`.
4. Build và nạp firmware.

## Bắt buộc

- Gọi `MotorController_Update(&motor_controller)` ở mỗi vòng `while(1)`.
- Nếu dùng watchdog, phải gửi command mới định kỳ (hoặc profile active).
- Để dùng PWM max theo `ARR`, giữ `MotorController_SetMaxPwm(&motor_controller, 0)`.

## Gợi ý tinh chỉnh nhanh

- Đảo chiều bánh: `MotorController_SetInversion(&motor_controller, left, right)`.
- Deadtime đảo chiều: `MotorController_SetDirectionSafety(&motor_controller, deadtime_ms, brake_ms)`.
- Min force để khởi động: `MotorController_SetMinDrivePercent(&motor_controller, min_start, min_run)`.
- `MOTOR_STOP_COAST`: thả trôi motor.
- `MOTOR_STOP_BRAKE`: hãm chủ động khi cần dừng nhanh.

Để chạy xe cân bằng, không dùng các profile chuyển động mẫu này làm vòng điều khiển
chính. Hãy xem `Examples/BalanceTests` và `Core/Src/app/app_main.cpp`.
