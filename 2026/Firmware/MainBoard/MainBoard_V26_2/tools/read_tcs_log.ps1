# MainBoard の RAM ロガー (src/control/tcs_log.c) の記録を ST-Link で読み、CSV にする。
# 列は TcsLog_DumpStep (UART) の出力と同じ。走行後、機体が止まってから ST-Link をつないで使う。
#   powershell -ExecutionPolicy Bypass -File tools\read_tcs_log.ps1                 # logs\tcs_ram_日時.csv
#   powershell -ExecutionPolicy Bypass -File tools\read_tcs_log.ps1 -Out logs\a.csv
param(
  [string]$Out,
  [string]$Elf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kSampleSize = 62  # sizeof(TcsLogSample)
$gdbSize = Get-GdbValue $Elf "sizeof('tcs_log.c'::samples[0])"
if ($gdbSize -ne $kSampleSize) { throw "TcsLogSample の大きさが $gdbSize byte です ($kSampleSize を想定)。デコードを直してください" }

$samplesAddr = Get-SymbolAddress $Elf "'tcs_log.c'::samples"
$countAddr = Get-SymbolAddress $Elf "'tcs_log.c'::sample_count"
$n = [int](Read-Ram16 $countAddr)
Write-Host ("samples @0x{0:X8}, sample_count = {1}" -f $samplesAddr, $n)
if ($n -eq 0) { Write-Host "記録がありません"; return }

$bytes = Read-RamBytes $samplesAddr ($n * $kSampleSize)

if (-not $Out) {
  $logDir = Join-Path (Split-Path $PSScriptRoot -Parent) "logs"
  $Out = Join-Path $logDir ("tcs_ram_{0}.csv" -f (Get-Date -Format "yyyyMMdd_HHmmss"))
}
$Out = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Out)
New-Item -ItemType Directory -Force (Split-Path $Out -Parent) | Out-Null

$header = "t_ms,tcs_on,slip,target_vx,cmd_vx,odom_vx,odom_vy,ground_vx,ground_vy," +
          "a_odom_x_cm,a_imu_x_cm,accel_res_cm,geom_res_x10,rot_res_mrad,accel_gain_x1000," +
          "w0_x100,w1_x100,w2_x100,w3_x100,cmd_vy,cmd_w_mrad,gyro_mrad," +
          "t0_x100,t1_x100,t2_x100,t3_x100,rx_stall,rx_restart,v0_x100,v1_x100,v2_x100,v3_x100"
$lines = New-Object System.Collections.Generic.List[string]
$lines.Add($header)
for ($k = 0; $k -lt $n; $k++) {
  $o = $k * $kSampleSize
  $cols = New-Object System.Collections.Generic.List[string]
  $cols.Add([string][BitConverter]::ToUInt16($bytes, $o))       # t_ms
  $cols.Add([string]$bytes[$o + 2])                              # tcs_on (区間の番号)
  $cols.Add([string]$bytes[$o + 3])                              # slip
  for ($j = 0; $j -lt 23; $j++) { $cols.Add([string][BitConverter]::ToInt16($bytes, $o + 4 + 2 * $j)) }
  $cols.Add([string]$bytes[$o + 50])                             # rx_stall
  $cols.Add([string][BitConverter]::ToUInt16($bytes, $o + 52))   # rx_restart
  for ($j = 0; $j -lt 4; $j++) { $cols.Add([string][BitConverter]::ToInt16($bytes, $o + 54 + 2 * $j)) }
  $lines.Add([string]::Join(",", $cols))
}
[System.IO.File]::WriteAllLines($Out, $lines, [System.Text.Encoding]::ASCII)

$tFirst = [BitConverter]::ToUInt16($bytes, 0)
$tLast = [BitConverter]::ToUInt16($bytes, ($n - 1) * $kSampleSize)
Write-Host ("{0} サンプル ({1:F1}〜{2:F1} 秒) -> {3}" -f $n, ($tFirst / 1000.0), ($tLast / 1000.0), $Out)
