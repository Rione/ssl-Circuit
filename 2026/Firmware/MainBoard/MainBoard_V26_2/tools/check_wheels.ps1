# 4輪の WheelUnit の報告 (実測の車輪速度・状態バイト・受信フレーム数) を ST-Link で読んで表示する (HANDOFF_AUTOTUNE.md 11.3)。
# 走らせる前に、4輪とも実測の車輪速度が動くことを確かめるために使う。
#   powershell -ExecutionPolicy Bypass -File tools\check_wheels.ps1             # 1回読む
#   powershell -ExecutionPolicy Bypass -File tools\check_wheels.ps1 -Count 20   # 0.5秒ごとに20回 (この間に車輪を手で回す/浮かせて回す)
# 見分け方: 24V を入れ忘れると meas が4輪ともちょうど 0。1輪だけ meas が常に 0 なら、その WheelUnit の FW 不良 (11.3)。
# 4輪とも、車輪を手で回したときに meas が動き、frames が増えていれば OK。
param(
  [int]$Count = 1,
  [int]$IntervalMs = 500,
  [switch]$Fast,   # 速度だけを読む (1回約1秒)。短く回しても取りこぼしにくい。status/frames/restarts は最初の1回だけ
  [string]$Elf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$speedAddr = Get-SymbolAddress $Elf "robot.omni_drive.vel_wheel_angular"    # float[4]
$statusAddr = Get-SymbolAddress $Elf "robot.omni_drive.wheel_status"        # uint8[4]
$framesAddr = Get-SymbolAddress $Elf "robot.omni_drive.wheel_frame_count"   # uint16[4]
$restartAddr = Get-SymbolAddress $Elf "robot.omni_drive.wheel_rx_restart_count"  # uint16[4]
$battAddr = Get-SymbolAddress $Elf "robot.info.battery_voltage"  # uint8 [V] (アドレスはビルドごとに変わるので、毎回 elf から引く)

$maxAbs = @(0.0, 0.0, 0.0, 0.0)
$firstFrames = $null
Write-Host "wheel:        ID1        ID2        ID3        ID4   (meas [rad/s] / status / frames / restarts)"
for ($n = 0; $n -lt $Count; $n++) {
  # アドレスが4byte境界とは限らない (uint8/uint16 の配列) ので、バイト単位で読む
  $sb = Read-RamBytes $speedAddr 16
  $meas = @(0..3 | ForEach-Object { [BitConverter]::ToSingle($sb, 4 * $_) })
  if ($Fast -and $n -gt 0) {
    for ($i = 0; $i -lt 4; $i++) { if ([math]::Abs($meas[$i]) -gt $maxAbs[$i]) { $maxAbs[$i] = [math]::Abs($meas[$i]) } }
    Write-Host ("meas    : " + (($meas | ForEach-Object { "{0,10:F2}" -f $_ }) -join " "))
    continue
  }
  $status = @((Read-RamBytes $statusAddr 4))
  $fb = Read-RamBytes $framesAddr 8
  $frames = @(0..3 | ForEach-Object { [int][BitConverter]::ToUInt16($fb, 2 * $_) })
  $rb = Read-RamBytes $restartAddr 8
  $restarts = @(0..3 | ForEach-Object { [int][BitConverter]::ToUInt16($rb, 2 * $_) })
  if ($null -eq $firstFrames) { $firstFrames = $frames }
  for ($i = 0; $i -lt 4; $i++) { if ([math]::Abs($meas[$i]) -gt $maxAbs[$i]) { $maxAbs[$i] = [math]::Abs($meas[$i]) } }
  Write-Host ("meas    : " + (($meas | ForEach-Object { "{0,10:F2}" -f $_ }) -join " "))
  Write-Host ("status  : " + (($status | ForEach-Object { "{0,10}" -f $_ }) -join " "))
  Write-Host ("frames  : " + (($frames | ForEach-Object { "{0,10}" -f $_ }) -join " "))
  Write-Host ("restarts: " + (($restarts | ForEach-Object { "{0,10}" -f $_ }) -join " "))
  if (-not $Fast) { Write-Host ("battery: {0,10} V" -f (Read-RamBytes $battAddr 1)[0]) }
  if ($n -lt $Count - 1) { Start-Sleep -Milliseconds $IntervalMs }
}

if ($Count -gt 1) {
  Write-Host ""
  Write-Host ("max|meas|: " + (($maxAbs | ForEach-Object { "{0,10:F2}" -f $_ }) -join " "))
  for ($i = 0; $i -lt 4; $i++) {
    $verdict = if ($maxAbs[$i] -lt 2.0) { "NG (meas が動かない。回さなかったか、24V 未投入か、この WheelUnit の FW 不良)" } else { "OK" }
    Write-Host ("ID{0}: {1}" -f ($i + 1), $verdict)
  }
}
