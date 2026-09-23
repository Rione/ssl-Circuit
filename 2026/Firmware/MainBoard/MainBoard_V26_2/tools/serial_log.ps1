# MainBoard の USART1 (PA9/PA10, 250000bps 8N1) の出力をファイルに保存しつつ画面にも表示する。
# 使い方:
#   powershell -ExecutionPolicy Bypass -File tools\serial_log.ps1 -List
#   powershell -ExecutionPolicy Bypass -File tools\serial_log.ps1 -Port COM3 -Seconds 60
# -Seconds を 0 にすると Ctrl+C まで記録し続ける。
param(
  [string]$Port,
  [int]$Baud = 250000,
  [int]$Seconds = 60,
  [string]$Out,
  [switch]$List
)

if ($List) {
  [System.IO.Ports.SerialPort]::GetPortNames()
  return
}
if (-not $Port) {
  Write-Error "-Port を指定してください (一覧は -List)"
  exit 1
}

$ErrorActionPreference = "Stop"

if (-not $Out) {
  $logDir = Join-Path (Split-Path $PSScriptRoot -Parent) "logs"
  $Out = Join-Path $logDir ("serial_{0}.csv" -f (Get-Date -Format "yyyyMMdd_HHmmss"))
}
# StreamWriter は PowerShell のカレントではなくプロセスのカレントで相対パスを解決するため、絶対パスにする
$Out = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Out)
New-Item -ItemType Directory -Force (Split-Path $Out -Parent) | Out-Null

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$sp.ReadBufferSize = 65536
$writer = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::ASCII)
$sp.Open()
Write-Host "Logging $Port @ $Baud bps -> $Out"

$sw = [System.Diagnostics.Stopwatch]::StartNew()
try {
  while ($Seconds -le 0 -or $sw.Elapsed.TotalSeconds -lt $Seconds) {
    $text = $sp.ReadExisting()
    if ($text) {
      $writer.Write($text)
      $writer.Flush()
      Write-Host -NoNewline $text
    }
    Start-Sleep -Milliseconds 20
  }
} finally {
  $writer.Close()
  $sp.Close()
  Write-Host "`nSaved: $Out"
}
