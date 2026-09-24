# ST-Link から MainBoard に「テストを1回走らせる」指示を書く (src/control/auto_tune.h)。
# 使い方 (ST-Link をつなぎ、機体は床に置いて電源を入れておく。Rock5A からの指令が無い状態):
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1            # 動作パターンを1回
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -TestId 2  # ランプ試験 (向きごとの限界、結果は read_ramp_results.ps1)
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -Status    # 状態を見るだけ
# 書いたら ST-Link を抜いて機体から離れる。10秒後 (LED0 が速く点滅している間) に走り出す。
# 走り終えたら、ST-Link をつないで tools\read_tcs_log.ps1 で記録を読む。
param(
  [int]$TestId = 1,        # 1: 動作パターンのテスト、2: ランプ試験 (AUTOTUNE_TEST_*)
  # この1回だけの volt_tune の上書き (省略 = 既定値。範囲は機体側で収める。走り終わると既定値に戻る)
  [double]$TractionV = 0,      # トルク上限 [V] 例: 2.4
  [double]$MaxAccel = 0,       # S字の加速度上限 [m/s^2]
  [double]$MaxAngAccel = 0,    # S字の角加速度上限 [rad/s^2]
  [string]$Elf,
  [switch]$Status
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kMagic = [uint32]0x41545331  # "ATS1" (AUTOTUNE_CTRL_MAGIC)
$stateNames = @{ 0 = "IDLE"; 1 = "WAITING"; 2 = "RUNNING" }
$resultNames = @{ 0 = "NONE"; 1 = "FINISHED"; 2 = "ABORTED (安全停止)"; 3 = "CANCELLED (Rock5A)"; 4 = "BAD_TEST" }

# AutoTuneCtrl: magic, start_seq, done_seq, test_id, state, result, traction_x100, max_accel_x100, max_ang_accel_x100 (すべて uint32)
$base = Get-SymbolAddress $Elf "autotune_ctrl"
function Show-Ctrl([uint32[]]$w) {
  Write-Host ("autotune_ctrl @0x{0:X8}: magic=0x{1:X8} start_seq={2} done_seq={3} test_id={4} state={5} result={6} override(x100)=[traction {7}, accel {8}, ang_accel {9}]" -f `
      $base, $w[0], $w[1], $w[2], $w[3], $stateNames[[int]$w[4]], $resultNames[[int]$w[5]], $w[6], $w[7], $w[8])
}

$ctrl = Read-Ram32 $base 9
Show-Ctrl $ctrl
if ($Status) { return }

if ($ctrl[4] -ne 0) {
  Write-Error "MainBoard は待機中または走行中です (state=$($stateNames[[int]$ctrl[4]]))。終わってから指示してください"
  exit 1
}

if ($ctrl[2] -eq [uint32]::MaxValue) { $seq = [uint32]0 } else { $seq = [uint32]($ctrl[2] + 1) }
# test_id → magic → start_seq の順に書く (MainBoard は magic と start_seq≠done_seq を見て始める)
$writes = @(@(($base + 12), $TestId),
  @(($base + 24), [uint32][math]::Round($TractionV * 100)),
  @(($base + 28), [uint32][math]::Round($MaxAccel * 100)),
  @(($base + 32), [uint32][math]::Round($MaxAngAccel * 100)),
  @($base, $kMagic), @(($base + 4), $seq))
Write-Ram32 $writes

$ctrl = Read-Ram32 $base 9
Show-Ctrl $ctrl
if ($ctrl[0] -ne $kMagic -or $ctrl[3] -ne $TestId -or ($ctrl[1] -ne $seq -and $ctrl[2] -ne $seq)) {
  Write-Error "書いた値を読み戻せませんでした。ELF と書き込み済みの FW が同じか確かめてください"
  exit 1
}
Write-Host ""
Write-Host "開始を指示しました (seq=$seq)。ST-Link を抜いて機体から離れてください。10秒後に走り出します。" -ForegroundColor Yellow
