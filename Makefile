SHELL := cmd.exe
.SHELLFLAGS := /C

PORT ?= 8000
COM ?= COM22
BAUD ?= 115200
READ ?= 2
SECS ?= 15
TOOLS_DIR := Tools\Visualizer

.PHONY: help visualizer uart-get uart-send uart-enable uart-disable autotest

help:
	@echo Targets:
	@echo   make visualizer         - Mo IMU Visualizer (http://localhost:$(PORT)/imu_visualizer.html)
	@echo   PORT=^<n^> co the dung de doi cong, vi du: make visualizer PORT=8080
	@echo   make uart-get           - Gui "BAL GET" qua UART, doc cau hinh hien tai
	@echo   make uart-send CMD="..." - Gui lenh UART tuy y, vd CMD="BAL APPLY kp=6 kd=0.18"
	@echo   make uart-enable        - Gui "BAL ENABLE 1"
	@echo   make uart-disable       - Gui "BAL ENABLE 0"
	@echo   make autotest CMD="BAL APPLY ..." SECS=15 - Tat/Apply/Bat/Ghi log/Phan tich tu dong
	@echo   COM=^<port^> BAUD=^<n^> READ=^<giay^> co the doi khi goi cac lenh uart-*

visualizer:
	@echo Visualizer: http://localhost:$(PORT)/imu_visualizer.html
	@echo Nhan Ctrl+C de dung server.
	@start "" "http://localhost:$(PORT)/imu_visualizer.html"
	@cd $(TOOLS_DIR) && (where python >nul 2>&1 && python -m http.server $(PORT) --bind 127.0.0.1 || py -3 -m http.server $(PORT) --bind 127.0.0.1)

uart-get:
	@powershell -NoProfile -ExecutionPolicy Bypass -File "$(TOOLS_DIR)\uart_cli.ps1" -Port $(COM) -Baud $(BAUD) -Command "BAL GET" -ReadSeconds $(READ)

uart-enable:
	@powershell -NoProfile -ExecutionPolicy Bypass -File "$(TOOLS_DIR)\uart_cli.ps1" -Port $(COM) -Baud $(BAUD) -Command "BAL ENABLE 1" -ReadSeconds $(READ)

uart-disable:
	@powershell -NoProfile -ExecutionPolicy Bypass -File "$(TOOLS_DIR)\uart_cli.ps1" -Port $(COM) -Baud $(BAUD) -Command "BAL ENABLE 0" -ReadSeconds $(READ)

uart-send:
	@powershell -NoProfile -ExecutionPolicy Bypass -File "$(TOOLS_DIR)\uart_cli.ps1" -Port $(COM) -Baud $(BAUD) -Command "$(CMD)" -ReadSeconds $(READ)

autotest:
	@powershell -NoProfile -ExecutionPolicy Bypass -File "$(TOOLS_DIR)\uart_autotest.ps1" -Port $(COM) -Baud $(BAUD) -ApplyCmd "$(CMD)" -Seconds $(SECS)
