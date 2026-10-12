<#
.SYNOPSIS
  Tu dong: tat balance -> ap dung cau hinh BAL APPLY -> bat balance -> ghi log N giay
  -> phan tich nhanh (bien do loi, % bao hoa PWM, fault). Khong can thao tac tay qua
  nhieu buoc rieng le.

.EXAMPLE
  .\uart_autotest.ps1 -ApplyCmd "BAL APPLY alg=PID kp=7 ki=0.001 kd=0.14 ag=5 rg=0.12 sg=0 pg=0 target=-1.6 max=100 boost_start=1 boost_max=100 cutoff=70 recover=10 invert=0" -Seconds 15
#>
param(
    [string]$Port = "COM22",
    [int]$Baud = 115200,
    [Parameter(Mandatory = $true)][string]$ApplyCmd,
    [int]$Seconds = 15,
    [string]$OutDir = "$env:TEMP\claude"
)

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$logFile = Join-Path $OutDir "autotest_$stamp.log"

$port_ = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port_.ReadTimeout = 500
$port_.NewLine = "`n"

function Send-Line([string]$text) {
    $port_.WriteLine($text)
}

function Drain-For([double]$seconds, [System.Text.StringBuilder]$sb) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $seconds) {
        try {
            $line = $port_.ReadLine()
            [void]$sb.AppendLine($line)
        } catch [System.TimeoutException] { continue }
    }
}

try {
    $port_.Open()
} catch {
    Write-Error "Khong mo duoc $Port : $($_.Exception.Message)"
    exit 1
}

$sb = New-Object System.Text.StringBuilder
try {
    Write-Host "1) Tat balance..." -ForegroundColor Cyan
    Send-Line "BAL ENABLE 0"
    Drain-For 1.0 $sb

    Write-Host "2) Ap dung cau hinh..." -ForegroundColor Cyan
    Send-Line $ApplyCmd
    Drain-For 1.0 $sb

    Write-Host "3) Bat balance va ghi log $Seconds giay..." -ForegroundColor Cyan
    Send-Line "BAL ENABLE 1"
    Drain-For $Seconds $sb
} finally {
    if ($port_.IsOpen) { $port_.Close() }
}

$text = $sb.ToString()
$text | Set-Content -Path $logFile -Encoding utf8

# --- Phan tich nhanh ---
$balLines = $text -split "`n" | Where-Object { $_ -match '\[BAL\]' -and $_ -match 'err_cdeg=' }
$errs = @(); $pwmLs = @(); $faults = @()
foreach ($l in $balLines) {
    if ($l -match 'err_cdeg=(-?\d+)') { $errs += [int]$Matches[1] }
    if ($l -match 'pwmL_percent=(-?\d+)') { $pwmLs += [int]$Matches[1] }
    if ($l -match 'fault=0x(\d+)') { $faults += [int]$Matches[1] }
}

Write-Host ""
Write-Host "=== Ket qua ($logFile) ===" -ForegroundColor Yellow
Write-Host ("So mau BAL: {0}" -f $balLines.Count)
if ($errs.Count -gt 0) {
    $minErr = ($errs | Measure-Object -Minimum).Minimum / 100.0
    $maxErr = ($errs | Measure-Object -Maximum).Maximum / 100.0
    Write-Host ("Sai so goc: {0:N2} deg .. {1:N2} deg" -f $minErr, $maxErr)
}
if ($pwmLs.Count -gt 0) {
    $satCount = ($pwmLs | Where-Object { [Math]::Abs($_) -ge 95 }).Count
    $satPct = [Math]::Round(100.0 * $satCount / $pwmLs.Count, 1)
    Write-Host ("PWM bao hoa (>=95%): {0}% so mau" -f $satPct)
}
$tiltFaultCount = ($faults | Where-Object { ($_ -band 4) -ne 0 }).Count
Write-Host ("So mau fault TILT_LIMIT: {0} / {1}" -f $tiltFaultCount, $faults.Count)
if ($tiltFaultCount -gt 0) {
    Write-Host "=> Xe cham cutoff (nga/vuot gioi han) trong qua trinh test." -ForegroundColor Red
} else {
    Write-Host "=> Khong cham tilt cutoff trong qua trinh test." -ForegroundColor Green
}
