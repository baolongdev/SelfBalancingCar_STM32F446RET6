<#
.SYNOPSIS
  Gui lenh text qua UART toi board STM32 (vd: BAL APPLY ..., BAL GET, BAL ENABLE 1)
  va in lai cac dong phan hoi, khong can nap lai firmware.

.EXAMPLE
  .\uart_cli.ps1 -Command "BAL GET"
  .\uart_cli.ps1 -Command "BAL APPLY alg=PID kp=6 ki=0.001 kd=0.18 ag=5 rg=0.12 sg=0 pg=0 target=0 max=100 boost_start=1 boost_max=100 cutoff=70 recover=10 invert=0"
  .\uart_cli.ps1 -Port COM22 -Command "BAL ENABLE 0" -ReadSeconds 1
#>
param(
    [string]$Port = "COM22",
    [int]$Baud = 115200,
    [Parameter(Mandatory = $true)][string]$Command,
    [double]$ReadSeconds = 2.0
)

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 300
$serial.NewLine = "`n"

try {
    $serial.Open()
} catch {
    Write-Error "Khong mo duoc $Port : $($_.Exception.Message)"
    exit 1
}

try {
    $serial.WriteLine($Command)
    Write-Host "> $Command" -ForegroundColor Cyan

    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $ReadSeconds) {
        try {
            $line = $serial.ReadLine()
            Write-Host $line
        } catch [System.TimeoutException] {
            continue
        }
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
