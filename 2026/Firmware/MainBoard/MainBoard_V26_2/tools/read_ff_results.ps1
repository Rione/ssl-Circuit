# FF 試験 (ramp_test の -Speeds ff、src/control/ramp_test.h の FfStepResult) の結果を ST-Link で読み、
# 向きごとの「指令の加速度に対する実際の加速度の比」と、推奨の FF 係数 (ka) を表示する。
#   powershell -ExecutionPolicy Bypass -File tools\read_ff_results.ps1                       # 今の ka を 0.5 として計算
#   powershell -ExecutionPolicy Bypass -File tools\read_ff_results.ps1 -KaLin 0.5 -KaLat 0.7 # 試験で使った ka を渡す
#   powershell -ExecutionPolicy Bypass -File tools\read_ff_results.ps1 -Csv logs\ff.csv
# 考え方: PI を切って FF だけで、指令の加速度 a_cmd を出した。実際の加速度 (IMU) a_imu との比 r = a_imu / a_cmd。
#  r < 1: FF が足りない → ka を上げる。ka_new = ka × a_cmd / a_imu。
#  2つの加速度 (1.5, 2.5 m/s²) の直線 a_imu = s × a_cmd + c で見ると、傾き s で ka を直し (ka_new = ka / s)、
#  切片 c は、負荷分の電圧 (床の摩擦。wheel_voltage.c の kLoadVolt) の食い違いを表す。
param(
  [double]$KaLin = 0.5,   # 試験のときに使っていた前後の FF 係数
  [double]$KaLat = 0.5,   # 試験のときに使っていた左右の FF 係数
  [string]$Csv,
  [string]$Elf
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kSize = 16
$kMax = 48
$gdbSize = Get-GdbValue $Elf "sizeof(ff_results[0])"
if ($gdbSize -ne $kSize) { throw "FfStepResult の大きさが $gdbSize byte です ($kSize を想定)。デコードを直してください" }
$addr = Get-SymbolAddress $Elf "ff_results"
$countAddr = Get-SymbolAddress $Elf "ff_result_count"
$n = [int](Read-Ram16 $countAddr)
Write-Host ("ff_results @0x{0:X8}, 本数 = {1}" -f $addr, $n)
try {
  $ast = (Read-Ram32 (Get-SymbolAddress $Elf "ramp_abort_status") 1)[0]
  if ($ast -ne 0) {
    $abt = Read-RamBytes (Get-SymbolAddress $Elf "ramp_abort_batt") 2
    Write-Host ("WheelUnit の異常で止めたときの状態バイト: ID1={0} ID2={1} ID3={2} ID4={3} (bit1: 電源電圧範囲外、bit2: 過熱)、電池 {4} V" -f ($ast -band 0xFF), (($ast -shr 8) -band 0xFF), (($ast -shr 16) -band 0xFF), (($ast -shr 24) -band 0xFF), [BitConverter]::ToUInt16($abt, 0))
  }
} catch { }
if ($n -eq 0) { Write-Host "記録がありません"; return }
$n = [math]::Min($n, $kMax)
$b = Read-RamBytes $addr ($n * $kSize)

$dirNames = @("前(+x)", "後(-x)", "左(+y)", "右(-y)")
$rows = New-Object System.Collections.Generic.List[object]
for ($i = 0; $i -lt $n; $i++) {
  $o = $i * $kSize
  $dir = [int]$b[$o]
  $aNom = [BitConverter]::ToUInt16($b, $o + 2) / 100.0
  $aCmd = [BitConverter]::ToInt16($b, $o + 4) / 100.0
  $aImu = [BitConverter]::ToInt16($b, $o + 6) / 100.0
  $aOdom = [BitConverter]::ToInt16($b, $o + 8) / 100.0
  $win = [BitConverter]::ToUInt16($b, $o + 10)
  $valid = ([int]$b[$o + 15] -eq 1)
  $rows.Add([pscustomobject]@{
      no = $i + 1; 向き = $dirNames[$dir]; dir = $dir; 回 = [int]$b[$o + 1]; 指令上限 = $aNom; 指令実測 = $aCmd; IMU = $aImu; 車輪 = $aOdom
      比 = $(if ($valid -and $aCmd -gt 0.3) { [math]::Round($aImu / $aCmd, 2) } else { $null })
      窓ms = $win; 終速度 = [BitConverter]::ToInt16($b, $o + 12) / 1000.0; 状態 = [int]$b[$o + 14]
      メモ = $(if (-not $valid) { if ($win -eq 0) { "走らず(範囲不足) 必要 {0:F2} m と予測" -f ([BitConverter]::ToInt16($b, $o + 12) / 1000.0) } else { "窓が短い" } } else { "" })
    })
}
Write-Host ""
Write-Host "== 1本ごと (加速度 [m/s²]。比 = IMU / 指令実測。車輪 > IMU なら空転)"
$rows | Format-Table no, 向き, 回, 指令上限, 指令実測, IMU, 車輪, 比, 窓ms, 終速度, 状態, メモ -AutoSize | Out-String -Width 300 | Write-Host

# 向きごと: 指令の加速度 2段階の平均から、比と、直線 (傾き s、切片 c)
Write-Host "== 向きごと (有効な本の平均。2つの指令の加速度から、直線 IMU = s × 指令 + c を当てはめる)"
$fit = foreach ($d in 0..3) {
  $g = @($rows | Where-Object { $_.dir -eq $d -and $null -ne $_.比 })
  if ($g.Count -eq 0) { continue }
  $lv = @($g | Group-Object 指令上限 | Sort-Object { [double]$_.Name })
  $pts = @($lv | ForEach-Object { [pscustomobject]@{ x = ($_.Group | Measure-Object 指令実測 -Average).Average; y = ($_.Group | Measure-Object IMU -Average).Average; n = $_.Count } })
  $ratio = ($g | Measure-Object 比 -Average).Average
  $s = $null; $c = $null
  if ($pts.Count -ge 2 -and ($pts[-1].x - $pts[0].x) -gt 0.3) {
    $s = ($pts[-1].y - $pts[0].y) / ($pts[-1].x - $pts[0].x)
    $c = $pts[0].y - $s * $pts[0].x
  }
  $ka = if ($d -lt 2) { $KaLin } else { $KaLat }
  [pscustomobject]@{
    向き = $dirNames[$d]; 本数 = $g.Count; 比の平均 = [math]::Round($ratio, 2)
    傾きs = $(if ($null -ne $s) { [math]::Round($s, 2) } else { $null }); 切片c = $(if ($null -ne $c) { [math]::Round($c, 2) } else { $null })
    推奨ka_比から = [math]::Round($ka / [math]::Max($ratio, 0.2), 3)
    推奨ka_傾きから = $(if ($null -ne $s -and $s -gt 0.2) { [math]::Round($ka / $s, 3) } else { $null })
  }
}
$fit | Format-Table -AutoSize | Out-String -Width 300 | Write-Host

$fb = @($fit | Where-Object { $_.向き -in @("前(+x)", "後(-x)") })
$lt = @($fit | Where-Object { $_.向き -in @("左(+y)", "右(-y)") })
if ($fb.Count -gt 0) { Write-Host ("前後 (基準): 比の平均 {0:F2}。1 に近ければ、FF の考え方は合っている" -f (($fb | Measure-Object 比の平均 -Average).Average)) }
if ($lt.Count -gt 0) {
  $rl = ($lt | Measure-Object 比の平均 -Average).Average
  Write-Host ("左右: 比の平均 {0:F2}" -f $rl)
  Write-Host ("推奨 ka_lat (左右の平均、比から) = {0:F3} V/(m/s²)  (今の ka_lat = {1})" -f (($lt | Measure-Object 推奨ka_比から -Average).Average), $KaLat)
  $sl = @($lt | Where-Object { $null -ne $_.推奨ka_傾きから })
  if ($sl.Count -gt 0) { Write-Host ("推奨 ka_lat (左右の平均、傾きから) = {0:F3}" -f (($sl | Measure-Object 推奨ka_傾きから -Average).Average)) }
}
if ($fb.Count -gt 0) {
  Write-Host ("参考: 推奨 ka_lin (前後の平均、比から) = {0:F3}  (今の ka_lin = {1})" -f (($fb | Measure-Object 推奨ka_比から -Average).Average), $KaLin)
}
Write-Host "次: 推奨値で試験をもう一度 (autotune_start.ps1 -TestId 2 -Speeds ff -Rect 3.5,2.5 -Rotate -KaLat <推奨値>)。比が 1 に近づけば、parammeter.h の WHEEL_VOLT_KA_LAT_BODY に書く。"

if ($Csv) {
  $Csv = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Csv)
  New-Item -ItemType Directory -Force (Split-Path $Csv -Parent) | Out-Null
  $rows | Select-Object no, 向き, dir, 回, 指令上限, 指令実測, IMU, 車輪, 比, 窓ms, 終速度, 状態, メモ | Export-Csv -Path $Csv -NoTypeInformation -Encoding UTF8
  Write-Host "CSV を保存しました: $Csv"
}
