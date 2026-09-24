# ランプ試験 (src/control/ramp_test.h) の結果を ST-Link で読み、向きごとにまとめて、採用する値を計算して表示する。
# 走り終えて機体が止まってから ST-Link をつないで使う。電源を切ると消える。
#   powershell -ExecutionPolicy Bypass -File tools\read_ramp_results.ps1
#   powershell -ExecutionPolicy Bypass -File tools\read_ramp_results.ps1 -Csv logs\ramp.csv   # 1本ごとの結果を CSV に保存
# 波形は tools\read_tcs_log.ps1 で読める (tcs_on 列 = 何本目か、target_vx 列 = フィルタ後のトルク上限 [mV])。
param(
  [string]$Csv,
  [string]$Elf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kHeader = 16
$kStroke = 32
$kMaxStrokes = 30
$kSize = $kHeader + $kStroke * $kMaxStrokes
$gdbSize = Get-GdbValue $Elf "sizeof(ramp_result)"
if ($gdbSize -ne $kSize) { throw "RampRunResult の大きさが $gdbSize byte です ($kSize を想定)。デコードを直してください" }
$addr = Get-SymbolAddress $Elf "ramp_result"
$b = Read-RamBytes $addr $kSize

$dirNames = @("前(+x)", "後(-x)", "左(+y)", "右(-y)", "左前", "右後", "右前", "左後", "左回り", "右回り")
$reasonNames = @{ 0 = "-"; 1 = "滑り始め"; 2 = "上限まで滑らず"; 3 = "速度の上限"; 4 = "電圧の余裕なし"; 5 = "滑らず停止"; 6 = "時間切れ" }
$resultNames = @{ 0 = "走行中/未完"; 1 = "最後まで"; 2 = "安全停止"; 3 = "取り消し" }
$abortNames = @{ 0 = "-"; 1 = "範囲外"; 2 = "向きのずれ"; 3 = "時間切れ"; 4 = "WheelUnit の異常" }

$seq = [BitConverter]::ToUInt32($b, 0)
$result = [BitConverter]::ToUInt32($b, 4)
$n = [BitConverter]::ToUInt16($b, 8)
$retry = [BitConverter]::ToUInt16($b, 10)
$unstable = [BitConverter]::ToUInt16($b, 12)
$abort = [BitConverter]::ToUInt16($b, 14)
Write-Host ("ramp_result @0x{0:X8}: 走行 {1}、結果 = {2}、本数 = {3}、安全停止の理由 = {4}" -f $addr, $seq, $resultNames[[int]$result], $n, $abortNames[[int]$abort])
if ($seq -eq 0 -or $n -eq 0) { Write-Host "記録がありません"; return }

function Get-Phase([int]$o, [bool]$rot) {
  $scale = if ($rot) { 10.0 } else { 100.0 }
  [pscustomobject]@{
    onset = [BitConverter]::ToUInt16($b, $o) / 100.0
    peakV = [BitConverter]::ToUInt16($b, $o + 2) / 100.0
    peakA = [BitConverter]::ToInt16($b, $o + 4) / $scale
    onsetA = [BitConverter]::ToInt16($b, $o + 6) / $scale
    speed = [BitConverter]::ToInt16($b, $o + 8) / 1000.0
    reason = [int][BitConverter]::ToUInt16($b, $o + 10)
  }
}

$rows = New-Object System.Collections.Generic.List[object]
for ($i = 0; $i -lt [math]::Min($n, $kMaxStrokes); $i++) {
  $o = $kHeader + $i * $kStroke
  $dir = [int]$b[$o]
  $rot = $dir -ge 8
  $acc = Get-Phase ($o + 8) $rot
  $brk = Get-Phase ($o + 20) $rot
  $rows.Add([pscustomobject]@{
      no = $i + 1; dir = $dir; 向き = $dirNames[$dir]; 回 = [int]$b[$o + 1]; t_s = [BitConverter]::ToUInt16($b, $o + 4) / 1000.0
      加速_ピークV = $acc.peakV; 加速_ピーク加速度 = $acc.peakA; 加速_滑り始めV = $acc.onset; 加速_終わり = $reasonNames[$acc.reason]; 加速_速度 = $acc.speed
      減速_ピークV = $brk.peakV; 減速_ピーク加速度 = $brk.peakA; 減速_滑り始めV = $brk.onset; 減速_終わり = $reasonNames[$brk.reason]
      状態 = [int]$b[$o + 2]; 電池V = [int]$b[$o + 3]
      acc = $acc; brk = $brk
    })
}

Write-Host ""
Write-Host "== 1本ごと (加速度の単位: 並進 m/s²、旋回 rad/s²。速度: 並進 m/s、旋回 rad/s)"
$rows | Format-Table no, 向き, 回, t_s, 加速_ピークV, 加速_ピーク加速度, 加速_滑り始めV, 加速_終わり, 加速_速度, 減速_ピークV, 減速_ピーク加速度, 減速_滑り始めV, 減速_終わり, 状態, 電池V -AutoSize | Out-String -Width 400 | Write-Host

# 向きごと: 滑り始め (主な指標) とピーク。どちらも、くり返しの中の最小値を採用する
Write-Host "== 向きごと (採用はくり返しの最小値。* は3回目を測った向き、! は3回でも差が大きかった向き)"
Write-Host "   滑り始め = 車輪と IMU の加速度が離れ始めたトルク。ピーク = IMU の加速度が一番大きかったトルク (滑り始めのあとも上げて測る)"
$perDir = foreach ($d in 0..9) {
  $r = @($rows | Where-Object { $_.dir -eq $d })
  if ($r.Count -eq 0) { continue }
  $withOnset = @($r | Where-Object { $_.acc.onset -gt 0 })
  $mark = ""
  if ($retry -band (1 -shl $d)) { $mark += "*" }
  if ($unstable -band (1 -shl $d)) { $mark += "!" }
  $bOn = @($r | Where-Object { $_.brk.onset -gt 0 } | ForEach-Object { $_.brk.onset })
  [pscustomobject]@{
    dir = $d; 向き = $dirNames[$d] + $mark
    滑り始めV_各回 = ($r | ForEach-Object { if ($_.acc.onset -gt 0) { "{0:F2}" -f $_.acc.onset } else { "なし" } }) -join " / "
    滑り始めV_採用 = if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.onset } | Measure-Object -Minimum).Minimum } else { $null }
    滑り始めの加速度 = if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.onsetA } | Measure-Object -Minimum).Minimum } else { $null }
    ピークV_各回 = ($r | ForEach-Object { "{0:F2}" -f $_.acc.peakV }) -join " / "
    ピークV_採用 = if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.peakV } | Measure-Object -Minimum).Minimum } else { $null }
    ピーク加速度 = if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.peakA } | Measure-Object -Minimum).Minimum } else { $null }
    滑らなかった回 = $r.Count - $withOnset.Count
    減速_滑り始めV = if ($bOn.Count) { ($bOn | Measure-Object -Minimum).Minimum } else { $null }
  }
}
$perDir | Format-Table -AutoSize | Out-String -Width 400 | Write-Host

# 採用する値の候補 (全方向共通)。並進8向きのうち、滑り始めが見つかった向きの最小。トルク上限は 3.2V 以下
$lin = @($perDir | Where-Object { $_.dir -lt 8 -and $null -ne $_.滑り始めV_採用 })
if ($lin.Count -gt 0) {
  $on = $lin | Sort-Object 滑り始めV_採用 | Select-Object -First 1
  $pk = $lin | Sort-Object ピークV_採用 | Select-Object -First 1
  $onA = ($lin | Measure-Object 滑り始めの加速度 -Minimum).Minimum
  $pkA = ($lin | Measure-Object ピーク加速度 -Minimum).Minimum
  Write-Host ("候補1 (滑り始めで決める): トルク上限 {0:F2} V ({1})、加速度上限 {2:F2} m/s²" -f [math]::Min($on.滑り始めV_採用, 3.2), $on.向き, $onA)
  Write-Host ("候補2 (ピークで決める)  : トルク上限 {0:F2} V ({1})、加速度上限 {2:F2} m/s²" -f [math]::Min($pk.ピークV_採用, 3.2), $pk.向き, $pkA)
} else {
  Write-Host "並進で滑り始めが見つかった向きがありません"
}
$rotAll = @($perDir | Where-Object { $_.dir -ge 8 })
if ($rotAll.Count -gt 0 -and @($rotAll | Where-Object { $null -ne $_.滑り始めV_採用 }).Count -eq 0) {
  Write-Host "旋回: 滑る前に速度の上限か電圧の余裕に達した (角加速度の限界は滑りではない)。角加速度上限は手動の値のまま"
}
Write-Host "（参考: 手動調整の値はトルク上限 2.8V、加速度上限 5.0 m/s²、角加速度上限 38 rad/s²）"
if ($Csv) {
  $Csv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Csv)
  New-Item -ItemType Directory -Force (Split-Path $Csv -Parent) | Out-Null
  $rows | Select-Object no, dir, 向き, 回, t_s, 加速_ピークV, 加速_ピーク加速度, 加速_滑り始めV, 加速_終わり, 加速_速度, 減速_ピークV, 減速_ピーク加速度, 減速_滑り始めV, 減速_終わり, 状態, 電池V |
    Export-Csv -Path $Csv -NoTypeInformation -Encoding UTF8
  Write-Host "CSV を保存しました: $Csv"
}
