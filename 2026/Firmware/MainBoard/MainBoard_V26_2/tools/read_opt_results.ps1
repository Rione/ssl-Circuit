# 自動最適化 (autotune_start.ps1 -Optimize、src/control/optimizer.h の OptResult) の結果を ST-Link で読む。
#   powershell -ExecutionPolicy Bypass -File tools\read_opt_results.ps1
# 反復ごとの表 (使った ka、前後・左右の比、更新後の ka)、最終状態 (合格/失敗、保存したか)、今の volt_tune の ka を表示する。
param([string]$Elf)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kHead = 32
$kLog = 24
$kMax = 16
$size = Get-GdbValue $Elf "sizeof(opt_result)"
if ($size -ne ($kHead + $kLog * $kMax)) { throw "OptResult の大きさが $size byte です ($($kHead + $kLog * $kMax) を想定)。デコードを直してください" }
$addr = Get-SymbolAddress $Elf "opt_result"
$b = Read-RamBytes $addr $size

$resultNames = @{ 0 = "走行中/未実行"; 1 = "合格して保存した"; 2 = "合格 (保存は許可されていない)"; 3 = "収束しなかった"; 4 = "走行の安全停止"; 5 = "測れなかった・比が範囲外 (センサの異常の疑い)"; 6 = "電池電圧・WheelUnit の異常・過熱が解消しなかった"; 7 = "時間の上限"; 8 = "取り消し (Rock5A)"; 9 = "合格したが保存に失敗" }
$seq = [BitConverter]::ToUInt32($b, 0)
$state = [BitConverter]::ToUInt32($b, 4)
$res = [BitConverter]::ToUInt32($b, 8)
$elapsed = [BitConverter]::ToUInt32($b, 12)
$runs = [BitConverter]::ToUInt16($b, 16)
$ver = [BitConverter]::ToUInt16($b, 18)
$saved = [BitConverter]::ToUInt16($b, 20)
$flags = [BitConverter]::ToUInt16($b, 22)
$s0 = @([BitConverter]::ToUInt16($b, 24), [BitConverter]::ToUInt16($b, 26))
$f0 = @([BitConverter]::ToUInt16($b, 28), [BitConverter]::ToUInt16($b, 30))
Write-Host ("opt_result @0x{0:X8}: seq={1} {2} 経過 {3:F1} 秒 走行 {4} 回 (検証 {5} 回) flags=0x{6:X2} ({7}、基準 {8})" -f $addr, $seq, $(if ($state -eq 1) { "走行中" } else { "停止" }), ($elapsed / 1000.0), $runs, $ver, $flags, $(if ($flags -band 1) { "保存許可" } else { "保存なし" }), $(if ($flags -band 2) { "IMU" } else { "車輪" }))
if ($seq -eq 0) { Write-Host "最適化はまだ走っていません"; return }
Write-Host ("結果: {0}" -f $resultNames[[int]$res])
Write-Host ("ka_lin: {0:F3} → {1:F3}   ka_lat: {2:F3} → {3:F3}   フラッシュに保存: {4}" -f ($s0[0] / 1000.0), ($f0[0] / 1000.0), ($s0[1] / 1000.0), ($f0[1] / 1000.0), $(if ($saved) { "した" } else { "していない" }))

$rows = for ($i = 0; $i -lt [math]::Min($runs, $kMax); $i++) {
  $o = $kHead + $i * $kLog
  $rf = [BitConverter]::ToInt16($b, $o + 4) / 1000.0
  $rl = [BitConverter]::ToInt16($b, $o + 6) / 1000.0
  [pscustomobject]@{
    走行 = $i + 1
    種類 = $(if ([BitConverter]::ToUInt16($b, $o) -eq 1) { "検証" } else { "反復" })
    結果 = $(if ([BitConverter]::ToUInt16($b, $o + 2) -eq 2) { "安全停止" } else { "完走" })
    ka_lin = [BitConverter]::ToUInt16($b, $o + 8) / 1000.0
    ka_lat = [BitConverter]::ToUInt16($b, $o + 10) / 1000.0
    比_前後 = $rf
    比_左右 = $rl
    有効本数 = [BitConverter]::ToUInt16($b, $o + 12)
    時刻s = [math]::Round([BitConverter]::ToUInt32($b, $o + 16) / 1000.0, 1)
    次_ka_lin = [BitConverter]::ToUInt16($b, $o + 20) / 1000.0
    次_ka_lat = [BitConverter]::ToUInt16($b, $o + 22) / 1000.0
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200 | Write-Host

# 今 (実行中の) volt_tune。保存された値を読み込んでいれば、既定値もその値になる
try {
  $vt = Get-SymbolAddress $Elf "volt_tune"
  $off = Get-GdbValue $Elf "(unsigned)&((VoltTuneParams*)0)->ka_lin"
  $w = Read-Ram32 ($vt + $off) 2
  Write-Host ("今の volt_tune: ka_lin={0:F3} ka_lat={1:F3}" -f [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$w[0]), 0), [BitConverter]::ToSingle([BitConverter]::GetBytes([uint32]$w[1]), 0))
} catch { Write-Host "(volt_tune は読めませんでした)" }
