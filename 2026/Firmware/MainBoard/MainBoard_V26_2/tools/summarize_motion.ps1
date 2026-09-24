# 動作パターンのテストの記録 (tools\read_tcs_log.ps1 の CSV、または docs\data\motion_pattern_*.csv) を区間ごとに要約する。
# 2つ渡すと並べて比べる。
#   powershell -ExecutionPolicy Bypass -File tools\summarize_motion.ps1 -Csv logs\stage1_run1.csv
#   powershell -ExecutionPolicy Bypass -File tools\summarize_motion.ps1 -Csv logs\stage1_run1.csv -Ref docs\data\motion_pattern_2.8V_pd_final_20260923.csv
# 列: dur = 区間の所要時間 [s] (30ms 周期の記録から)、slip = is_slipping (bit7) の割合、lim = 出力を縮めた (bit6) の割合、
#     vmax = オドメトリの最高速度 [m/s]、wmax = 最大の車輪角速度 [rad/s]、gyro = ジャイロの最大 [rad/s]、
#     vmaxV = 印加電圧の最大 [V]
param(
  [Parameter(Mandatory = $true)][string]$Csv,
  [string]$Ref
)

$ErrorActionPreference = "Stop"

function Get-Summary([string]$Path) {
  $rows = Get-Content $Path | Where-Object { $_ -notmatch '^#' } | ConvertFrom-Csv
  $segCol = if ($rows[0].PSObject.Properties.Name -contains "seg") { "seg" } else { "tcs_on" }
  $groups = $rows | Group-Object { [int]$_.$segCol } | Sort-Object { [int]$_.Name }
  foreach ($g in $groups) {
    $r = @($g.Group)
    $n = $r.Count
    $t0 = [int]$r[0].t_ms
    $t1 = [int]$r[$n - 1].t_ms
    $slip = @($r | Where-Object { ([int]$_.slip -band 0x80) -ne 0 }).Count
    $lim = @($r | Where-Object { ([int]$_.slip -band 0x40) -ne 0 }).Count
    $vmax = ($r | ForEach-Object { [math]::Sqrt([math]::Pow([double]$_.odom_vx, 2) + [math]::Pow([double]$_.odom_vy, 2)) } | Measure-Object -Maximum).Maximum / 1000.0
    $wmax = ($r | ForEach-Object { @([math]::Abs([double]$_.w0_x100), [math]::Abs([double]$_.w1_x100), [math]::Abs([double]$_.w2_x100), [math]::Abs([double]$_.w3_x100)) | Measure-Object -Maximum | ForEach-Object Maximum } | Measure-Object -Maximum).Maximum / 100.0
    $gmax = ($r | ForEach-Object { [math]::Abs([double]$_.gyro_mrad) } | Measure-Object -Maximum).Maximum / 1000.0
    $vv = ($r | ForEach-Object { @([math]::Abs([double]$_.v0_x100), [math]::Abs([double]$_.v1_x100), [math]::Abs([double]$_.v2_x100), [math]::Abs([double]$_.v3_x100)) | Measure-Object -Maximum | ForEach-Object Maximum } | Measure-Object -Maximum).Maximum / 100.0
    [pscustomobject]@{
      seg = [int]$g.Name; t0 = $t0; dur = [math]::Round(($t1 - $t0 + 30) / 1000.0, 2)
      slip = [int](100 * $slip / $n); lim = [int](100 * $lim / $n)
      vmax = [math]::Round($vmax, 2); wmax = [math]::Round($wmax, 1); gyro = [math]::Round($gmax, 1); vmaxV = [math]::Round($vv, 2)
      n = $n
    }
  }
}

$names = @("前後1(+x)", "前後2(-x)", "左1(+y)", "右(-2y)", "左2(+y)", "斜め左前", "斜め右後", "斜め右前", "斜め左後", "旋回+π", "旋回-π", "旋回+前進", "旋回+後退")
$a = @(Get-Summary $Csv)
$b = if ($Ref) { @(Get-Summary $Ref) } else { @() }

function Show([object[]]$s, [string]$title) {
  Write-Host ""
  Write-Host $title
  $s | ForEach-Object { $_ | Add-Member -NotePropertyName name -NotePropertyValue $names[$_.seg - 1] -PassThru } |
    Format-Table seg, name, t0, dur, slip, lim, vmax, wmax, gyro, vmaxV -AutoSize | Out-String | Write-Host
  $total = ($s | Measure-Object dur -Sum).Sum
  $slipAvg = [math]::Round((($s | ForEach-Object { $_.slip * $_.n } | Measure-Object -Sum).Sum) / (($s | Measure-Object n -Sum).Sum), 0)
  Write-Host ("合計 dur = {0:F2} s、区間数 = {1}、slip(全体) = {2}%" -f $total, $s.Count, $slipAvg)
}

Show $a "== $Csv"
if ($b.Count -gt 0) {
  Show $b "== $Ref (参照)"
  Write-Host ""
  Write-Host "== 区間ごとの差 (今回 - 参照): dur[s]  slip[%pt]  vmax[m/s]"
  foreach ($x in $a) {
    $y = $b | Where-Object { $_.seg -eq $x.seg }
    if ($y) {
      Write-Host ("seg{0,2} {1,-8} {2,7:+0.00;-0.00}  {3,5:+0;-0}  {4,7:+0.00;-0.00}" -f $x.seg, $names[$x.seg - 1], ($x.dur - $y.dur), ($x.slip - $y.slip), ($x.vmax - $y.vmax))
    }
  }
}
