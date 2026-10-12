<#
.SYNOPSIS
  Gui (tuy chon) mot lenh, bat log nhanh [T] bang "BAL LOG <ms>", ghi N giay ra
  file, roi tat log nhanh. Dung cho tune PID: phan tich file bang tune_analyze.py.

.EXAMPLE
  .\uart_capture.ps1 -Seconds 5 -Out capture.log
  .\uart_capture.ps1 -Command "BAL APPLY kp=12 kd=0.4" -Seconds 6 -Out kp12.log
#>
param(
    [string]$Port = "COM22",
    [int]$Baud = 115200,
    [string]$Command = "",
    [int]$FastLogMs = 8,
    [double]$Seconds = 5,
    [Parameter(Mandatory = $true)][string]$Out
)

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 200
$serial.NewLine = "`n"
try { $serial.Open() } catch { Write-Error "Khong mo duoc $Port : $($_.Exception.Message)"; exit 1 }

$sb = New-Object System.Text.StringBuilder
function Drain([double]$sec) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $sec) {
        try { [void]$sb.AppendLine($serial.ReadLine().TrimEnd("`r")) } catch [System.TimeoutException] { }
    }
}

try {
    $serial.DiscardInBuffer()
    if ($Command -ne "") { $serial.WriteLine($Command); Drain 0.3 }
    $serial.WriteLine("BAL LOG $FastLogMs")
    Drain $Seconds
    $serial.WriteLine("BAL LOG 0")
    Drain 0.3
} finally {
    if ($serial.IsOpen) { $serial.Close() }
}
$sb.ToString() | Set-Content -Path $Out -Encoding utf8
Write-Host "Saved $Out"
