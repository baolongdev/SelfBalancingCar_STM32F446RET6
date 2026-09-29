$ErrorActionPreference = 'Stop'
$toolDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$port = 8000

if (Get-Command python -ErrorAction SilentlyContinue) {
    $pythonCommand = 'python'
} elseif (Get-Command py -ErrorAction SilentlyContinue) {
    $pythonCommand = 'py'
} else {
    throw 'Không tìm thấy Python. Hãy cài Python 3 và bật tùy chọn Add Python to PATH.'
}

Write-Host "Balance tuning: http://localhost:$port/balance_tuning.html"
Write-Host 'Nhấn Ctrl+C để dừng server.'
Start-Process "http://localhost:$port/balance_tuning.html"

if ($pythonCommand -eq 'py') {
    & py -3 -m http.server $port --bind 127.0.0.1 --directory $toolDir
} else {
    & python -m http.server $port --bind 127.0.0.1 --directory $toolDir
}
