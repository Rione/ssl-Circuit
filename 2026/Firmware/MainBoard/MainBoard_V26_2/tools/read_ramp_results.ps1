# ランプ試験 (src/control/ramp_test.h) の結果を ST-Link で読み、まとめて表示する。
# 走り終えて機体が止まってから ST-Link をつないで使う。電源を切ると消える。
#   powershell -ExecutionPolicy Bypass -File tools\read_ramp_results.ps1
#   powershell -ExecutionPolicy Bypass -File tools\read_ramp_results.ps1 -Csv logs\ramp.csv   # 1本ごとの結果を CSV に保存
# 波形は tools\read_tcs_log.ps1 で読める (tcs_on 列 = 何本目か、target_vx 列 = フィルタ後のトルク上限 [mV])。
#  速度別の測定のときは、波形のログは速度別の測定のぶんだけ (低速の試験のぶんは消える)。
param(
  [string]$Csv,
  [string]$Elf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kHeader = 20
$kStroke = 40
$kMaxStrokes = 56
$kSize = $kHeader + $kStroke * $kMaxStrokes
$gdbSize = Get-GdbValue $Elf "sizeof(ramp_result)"
if ($gdbSize -ne $kSize) { throw "RampRunResult の大きさが $gdbSize byte です ($kSize を想定)。デコードを直してください" }
$addr = Get-SymbolAddress $Elf "ramp_result"
$b = Read-RamBytes $addr $kSize

$dirNames = @("前(+x)", "後(-x)", "左(+y)", "右(-y)", "左前", "右後", "右前", "左後", "左回り", "右回り")
$reasonNames = @{ 0 = "-"; 1 = "滑り始め"; 2 = "上限まで"; 3 = "速度の上限"; 4 = "電圧の余裕なし"; 5 = "滑らず停止"; 6 = "時間切れ"; 7 = "範囲不足(走らず)" }
$resultNames = @{ 0 = "走行中/未完"; 1 = "最後まで"; 2 = "安全停止"; 3 = "取り消し" }
$abortNames = @{ 0 = "-"; 1 = "範囲外"; 2 = "向きのずれ"; 3 = "時間切れ"; 4 = "WheelUnit の異常" }

$seq = [BitConverter]::ToUInt32($b, 0)
$result = [BitConverter]::ToUInt32($b, 4)
$n = [BitConverter]::ToUInt16($b, 8)
$retry = [BitConverter]::ToUInt16($b, 10)
$unstable = [BitConverter]::ToUInt16($b, 12)
$abort = [BitConverter]::ToUInt16($b, 14)
$mask = [BitConverter]::ToUInt32($b, 16)
Write-Host ("ramp_result @0x{0:X8}: 走行 {1}、結果 = {2}、本数 = {3}、速度の段 mask = 0x{4:X2}、安全停止の理由 = {5}" -f $addr, $seq, $resultNames[[int]$result], $n, $mask, $abortNames[[int]$abort])
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
  $v0 = [BitConverter]::ToUInt16($b, $o + 6) / 100.0
  $vb = [BitConverter]::ToUInt16($b, $o + 32) / 1000.0
  $bd = [BitConverter]::ToUInt16($b, $o + 34) / 1000.0
  $rows.Add([pscustomobject]@{
      no = $i + 1; dir = $dir; 向き = $dirNames[$dir]; 回 = [int]$b[$o + 1]; t_s = [BitConverter]::ToUInt16($b, $o + 4) / 1000.0; v0 = $v0
      加速_ピークV = $acc.peakV; 加速_ピーク加速度 = $acc.peakA; 加速_滑り始めV = $acc.onset; 加速_終わり = $reasonNames[$acc.reason]; 加速_速度 = $acc.speed
      減速_ピークV = $brk.peakV; 減速_ピーク加速度 = $brk.peakA; 減速_滑り始めV = $brk.onset; 減速_終わり = $reasonNames[$brk.reason]
      ブレーキ開始速度 = $vb; 停止距離 = $bd; 平均減速度 = $(if ($bd -gt 0.05 -and $vb -gt 0.3) { [math]::Round($vb * $vb / (2 * $bd), 2) } else { $null })
      加速距離 = [BitConverter]::ToUInt16($b, $o + 36) / 1000.0; 助走距離 = [BitConverter]::ToUInt16($b, $o + 38) / 1000.0
      状態 = [int]$b[$o + 2]; 電池V = [int]$b[$o + 3]
      acc = $acc; brk = $brk
    })
}

# ---- 低速 (v0 = 0、止まった状態から) ----
$low = @($rows | Where-Object { $_.v0 -eq 0 })
if ($low.Count -gt 0) {
  Write-Host ""
  Write-Host "== 低速 (止まった状態から) 1本ごと (加速度: 並進 m/s²、旋回 rad/s²。速度: 並進 m/s、旋回 rad/s)"
  $low | Format-Table no, 向き, 回, t_s, 加速_ピークV, 加速_ピーク加速度, 加速_滑り始めV, 加速_終わり, 加速_速度, 減速_ピークV, 減速_ピーク加速度, 減速_滑り始めV, 減速_終わり, 状態, 電池V -AutoSize | Out-String -Width 400 | Write-Host

  Write-Host "== 低速: 向きごと (採用はくり返しの最小値。* は3回目を測った向き、! は3回でも差が大きかった向き)"
  Write-Host "   滑り始め = 車輪と IMU の加速度が離れ始めたトルク。ピーク = IMU の加速度が一番大きかったトルク (滑り始めのあとも上げて測る)"
  $perDir = foreach ($d in 0..9) {
    $g = @($low | Where-Object { $_.dir -eq $d })
    if ($g.Count -eq 0) { continue }
    $withOnset = @($g | Where-Object { $_.acc.onset -gt 0 })
    $mark = ""
    if ($retry -band (1 -shl $d)) { $mark += "*" }
    if ($unstable -band (1 -shl $d)) { $mark += "!" }
    $bOn = @($g | Where-Object { $_.brk.onset -gt 0 } | ForEach-Object { $_.brk.onset })
    [pscustomobject]@{
      dir = $d; 向き = $dirNames[$d] + $mark
      滑り始めV_各回 = ($g | ForEach-Object { if ($_.acc.onset -gt 0) { "{0:F2}" -f $_.acc.onset } else { "なし" } }) -join " / "
      滑り始めV_採用 = $(if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.onset } | Measure-Object -Minimum).Minimum } else { $null })
      滑り始めの加速度 = $(if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.onsetA } | Measure-Object -Minimum).Minimum } else { $null })
      ピークV_各回 = ($g | ForEach-Object { "{0:F2}" -f $_.acc.peakV }) -join " / "
      ピークV_採用 = $(if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.peakV } | Measure-Object -Minimum).Minimum } else { $null })
      ピーク加速度 = $(if ($withOnset.Count) { ($withOnset | ForEach-Object { $_.acc.peakA } | Measure-Object -Minimum).Minimum } else { $null })
      滑らなかった回 = $g.Count - $withOnset.Count
      減速_滑り始めV = $(if ($bOn.Count) { ($bOn | Measure-Object -Minimum).Minimum } else { $null })
    }
  }
  $perDir | Format-Table -AutoSize | Out-String -Width 400 | Write-Host

  $lin = @($perDir | Where-Object { $_.dir -lt 8 -and $null -ne $_.滑り始めV_採用 })
  if ($lin.Count -gt 0) {
    $onDir = $lin | Sort-Object 滑り始めV_採用 | Select-Object -First 1
    $pkDir = $lin | Sort-Object ピークV_採用 | Select-Object -First 1
    $onA = ($lin | Measure-Object 滑り始めの加速度 -Minimum).Minimum
    $pkA = ($lin | Measure-Object ピーク加速度 -Minimum).Minimum
    Write-Host ("候補1 (滑り始めで決める): トルク上限 {0:F2} V ({1})、加速度上限 {2:F2} m/s²" -f [math]::Min($onDir.滑り始めV_採用, 3.2), $onDir.向き, $onA)
    Write-Host ("候補2 (ピークで決める)  : トルク上限 {0:F2} V ({1})、加速度上限 {2:F2} m/s²" -f [math]::Min($pkDir.ピークV_採用, 3.2), $pkDir.向き, $pkA)
  }
  Write-Host "（参考: 手動調整の値はトルク上限 2.8V、加速度上限 5.0 m/s²、角加速度上限 38 rad/s²）"
}

# ---- 速度別 (v0 > 0) ----
$spd = @($rows | Where-Object { $_.v0 -gt 0 })
if ($spd.Count -gt 0) {
  Write-Host ""
  Write-Host "== 速度別 1本ごと (v0 = 巡航の速度 [m/s]。距離 [m]。平均減速度 = v²/(2×停止距離) [m/s²])"
  foreach ($x in $spd) {
    $note = if ($x.acc.reason -eq 7) { "範囲不足: 必要 {0:F2} m と予測" -f $x.助走距離 } elseif ($x.acc.reason -eq 0) { "ブレーキのみ" } else { "" }
    $x | Add-Member -NotePropertyName メモ -NotePropertyValue $note -Force
  }
  Write-Host "   回 = 速度別の何回目か (1: 1回目、2: 機体を 90° 回した2回目。向きは機体から見た向き)"
  $spd | Format-Table no, 回, 向き, v0, 加速_滑り始めV, 加速_ピークV, 加速_ピーク加速度, 加速_終わり, 加速_速度, 加速距離, 減速_滑り始めV, 減速_終わり, ブレーキ開始速度, 停止距離, 平均減速度, 助走距離, 電池V, メモ -AutoSize | Out-String -Width 400 | Write-Host

  Write-Host "== ブレーキの停止距離表 (速度 v0 と向きごと。ブレーキは弱い所から上げていくので、停止距離は最短ではなく上限側の値)"
  $spd | Where-Object { $_.acc.reason -ne 7 -and $_.停止距離 -gt 0 } | Sort-Object v0, dir |
    Format-Table 回, 向き, v0, ブレーキ開始速度, 停止距離, 平均減速度, 減速_ピーク加速度, 減速_ピークV, 減速_滑り始めV, 減速_終わり, メモ -AutoSize | Out-String -Width 300 | Write-Host

  # 加速とブレーキの IMU ピーク (機体が実際に出した加速度) の比較
  $both = @($spd | Where-Object { $_.acc.reason -ne 7 -and $_.acc.reason -ne 0 -and $_.加速_ピーク加速度 -gt 0 -and $_.減速_ピーク加速度 -gt 0 })
  if ($both.Count -gt 0) {
    $ma = ($both | Measure-Object 加速_ピーク加速度 -Average).Average; $mb = ($both | Measure-Object 減速_ピーク加速度 -Average).Average
    Write-Host ("== 加速と減速の IMU ピーク (加速+ブレーキの {0} 本の平均): 加速 {1:F2} m/s²、減速 {2:F2} m/s²" -f $both.Count, $ma, $mb)
  }
  Write-Host "== 加速側の限界の速度依存 (滑り始めのトルク [V] と、IMU の加速度ピーク [m/s²]。v0 の低い順)"
  foreach ($d in 0..7) {
    $g = @($spd | Where-Object { $_.dir -eq $d -and $_.acc.reason -ne 7 -and $_.acc.reason -ne 0 } | Sort-Object v0)
    if ($g.Count -eq 0) { continue }
    $parts = $g | ForEach-Object {
      $ons = if ($_.acc.onset -gt 0) { "{0:F2}V" -f $_.acc.onset } else { "なし" }
      "[{4}]v0={0:F1}: 滑り{1} ピーク{2:F1}m/s2 ({3})" -f $_.v0, $ons, $_.acc.peakA, $_.加速_終わり, $_.回
    }
    Write-Host ("  {0,-8}: {1}" -f $dirNames[$d], ($parts -join " | "))
  }
  $skipRows = @($spd | Where-Object { $_.acc.reason -eq 7 })
  if ($skipRows.Count -gt 0) {
    Write-Host ""
    Write-Host ("走らせなかった本 (範囲不足と予測): " + (($skipRows | ForEach-Object { "{0} v0={1:F1} (必要 {2:F2} m)" -f $_.向き, $_.v0, $_.助走距離 }) -join "、"))
  }
}

if ($Csv) {
  $Csv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Csv)
  New-Item -ItemType Directory -Force (Split-Path $Csv -Parent) | Out-Null
  $rows | Select-Object no, dir, 向き, 回, t_s, v0, 加速_ピークV, 加速_ピーク加速度, 加速_滑り始めV, 加速_終わり, 加速_速度, 加速距離, 助走距離, 減速_ピークV, 減速_ピーク加速度, 減速_滑り始めV, 減速_終わり, ブレーキ開始速度, 停止距離, 平均減速度, 状態, 電池V |
    Export-Csv -Path $Csv -NoTypeInformation -Encoding UTF8
  Write-Host "CSV を保存しました: $Csv"
}
