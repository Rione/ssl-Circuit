# MainBoard の RAM に残した動作パターンの区間ごとの要約 (src/control/motion_summary.h) を ST-Link で読み、表にする。
# 走行後、機体が止まってから ST-Link をつないで使う。電源を切ると消える (直近8回ぶん)。
#   powershell -ExecutionPolicy Bypass -File tools\read_motion_results.ps1             # 走行ごとの一覧 + 最新の1回の区間表
#   powershell -ExecutionPolicy Bypass -File tools\read_motion_results.ps1 -All         # すべての走行の区間表
#   powershell -ExecutionPolicy Bypass -File tools\read_motion_results.ps1 -Csv logs\runs.csv   # 全走行・全区間を CSV に保存
# 列の意味は motion_summary.h。s_acc / gap は候補のスリップ指標 (段階2で S_ref を決めるのに使う)。
param(
  [switch]$All,
  [string]$Csv,
  [string]$Elf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kRuns = 8
$kSegMax = 13
$kSegSize = 32
$kRunHeader = 20
$kRunSize = $kRunHeader + $kSegMax * $kSegSize  # 436
$gdbRunSize = Get-GdbValue $Elf "sizeof(motion_results[0])"
if ($gdbRunSize -ne $kRunSize) { throw "MotionRunResult の大きさが $gdbRunSize byte です ($kRunSize を想定)。デコードを直してください" }

$resultsAddr = Get-SymbolAddress $Elf "motion_results"
$countAddr = Get-SymbolAddress $Elf "motion_result_count"
$count = (Read-Ram32 $countAddr 1)[0]
Write-Host ("motion_results @0x{0:X8}, これまでに始めた走行 = {1}" -f $resultsAddr, $count)
if ($count -eq 0) { Write-Host "記録がありません"; return }

$bytes = Read-RamBytes $resultsAddr ($kRuns * $kRunSize)
$names = @("前後1(+x)", "前後2(-x)", "左1(+y)", "右(-2y)", "左2(+y)", "斜め左前", "斜め右後", "斜め右前", "斜め左後", "旋回+π", "旋回-π", "旋回+前進", "旋回+後退")
$resultNames = @{ 0 = "走行中"; 1 = "最後まで"; 2 = "安全停止"; 3 = "取り消し" }

$runs = New-Object System.Collections.Generic.List[object]
for ($k = 0; $k -lt $kRuns; $k++) {
  $o = $k * $kRunSize
  $seq = [BitConverter]::ToUInt32($bytes, $o)
  if ($seq -eq 0) { continue }
  $segs = New-Object System.Collections.Generic.List[object]
  $segCount = [BitConverter]::ToUInt16($bytes, $o + 14)
  for ($s = 0; $s -lt [math]::Min($segCount, $kSegMax); $s++) {
    $q = $o + $kRunHeader + $s * $kSegSize
    $u = { param($i) [int][BitConverter]::ToUInt16($bytes, $q + 2 * $i) }
    $i16 = { param($i) [int][BitConverter]::ToInt16($bytes, $q + 2 * $i) }
    $segs.Add([pscustomobject]@{
        seg = $s + 1; name = $names[$s]
        dur = (& $u 0) / 1000.0; slip = (& $u 1) / 10.0; lim = (& $u 2) / 10.0; vmax = (& $i16 3) / 1000.0
        s_acc = (& $i16 4) / 100.0; gap = (& $i16 5); gap_fr = (& $u 6) / 10.0; acc_ms = (& $u 7)
        cross = (& $i16 8); over = (& $i16 9); h_over = [math]::Round((& $i16 10) * 180.0 / 3141.59, 1)
        end_d = (& $i16 11); end_h = [math]::Round((& $i16 12) * 180.0 / 3141.59, 1)
        st = (& $u 13); batt = (& $u 14) / 10.0
      })
  }
  $runs.Add([pscustomobject]@{
      seq = $seq; result = $resultNames[[int][BitConverter]::ToUInt32($bytes, $o + 4)]
      traction = [BitConverter]::ToUInt16($bytes, $o + 8) / 100.0
      max_accel = [BitConverter]::ToUInt16($bytes, $o + 10) / 100.0
      max_ang = [BitConverter]::ToUInt16($bytes, $o + 12) / 100.0
      total = [BitConverter]::ToUInt32($bytes, $o + 16) / 1000.0
      segs = $segs
    })
}
$runs = @($runs | Sort-Object seq)

Write-Host ""
Write-Host "== 走行の一覧 (電源投入からの通し番号)"
$runs | ForEach-Object {
  $r = $_
  $sl = if ($r.segs.Count) { ($r.segs | Measure-Object slip -Average).Average } else { 0 }
  $sa = if ($r.segs.Count) { ($r.segs | Where-Object { $_.acc_ms -gt 0 } | Measure-Object s_acc -Average).Average } else { 0 }
  [pscustomobject]@{ seq = $r.seq; result = $r.result; トルク上限V = $r.traction; accel = $r.max_accel; ang = $r.max_ang; 区間数 = $r.segs.Count; 合計s = [math]::Round($r.total, 2); slip平均 = [math]::Round($sl, 0); s_acc平均 = [math]::Round($sa, 2) }
} | Format-Table -AutoSize | Out-String | Write-Host

$show = if ($All) { $runs } else { @($runs[-1]) }
foreach ($r in $show) {
  Write-Host ("== 走行 {0} ({1})  トルク上限 {2}V  加速度 {3}  角加速度 {4}  合計 {5:F2} 秒" -f $r.seq, $r.result, $r.traction, $r.max_accel, $r.max_ang, $r.total)
  Write-Host "   dur[s] slip/lim[%] vmax[m/s] s_acc[m/s2] gap[mm/s] gap_fr[%] cross/over/end_d[mm] h_over/end_h[deg] status batt[V]"
  $r.segs | Format-Table seg, name, dur, slip, lim, vmax, s_acc, gap, gap_fr, cross, over, end_d, h_over, end_h, st, batt -AutoSize | Out-String | Write-Host
}

if ($Csv) {
  $Csv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Csv)
  New-Item -ItemType Directory -Force (Split-Path $Csv -Parent) | Out-Null
  $runs | ForEach-Object { $r = $_; $r.segs | ForEach-Object { [pscustomobject]@{ run = $r.seq; result = $r.result; traction = $r.traction; max_accel = $r.max_accel; max_ang = $r.max_ang; seg = $_.seg; dur = $_.dur; slip = $_.slip; lim = $_.lim; vmax = $_.vmax; s_acc = $_.s_acc; acc_ms = $_.acc_ms; gap = $_.gap; gap_fr = $_.gap_fr; cross = $_.cross; over = $_.over; h_over = $_.h_over; end_d = $_.end_d; end_h = $_.end_h; st = $_.st; batt = $_.batt } } } |
    Export-Csv -Path $Csv -NoTypeInformation -Encoding UTF8
  Write-Host "CSV を保存しました: $Csv"
}
