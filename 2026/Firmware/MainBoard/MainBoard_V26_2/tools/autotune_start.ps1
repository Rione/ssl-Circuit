# ST-Link から MainBoard に「テストを1回走らせる」指示を書く (src/control/auto_tune.h)。
# 使い方 (ST-Link をつなぎ、機体は床に置いて電源を入れておく。Rock5A からの指令が無い状態):
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1            # 動作パターンを1回
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -Status    # 状態を見るだけ
# 書いたら ST-Link を抜いて機体から離れる。10秒後 (LED0 が速く点滅している間) に走り出す。
# 走り終えたら、ST-Link をつないで tools\read_tcs_log.ps1 で記録を読む。
param(
  [int]$TestId = 1,        # 1: 動作パターンのテスト (AUTOTUNE_TEST_MOTION_PATTERN)
  [string]$Elf,
  [switch]$Status
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kMagic = [uint32]0x41545331  # "ATS1" (AUTOTUNE_CTRL_MAGIC)
$stateNames = @{ 0 = "IDLE"; 1 = "WAITING"; 2 = "RUNNING" }
$resultNames = @{ 0 = "NONE"; 1 = "FINISHED"; 2 = "ABORTED (安全停止)"; 3 = "CANCELLED (Rock5A)"; 4 = "BAD_TEST" }

# AutoTuneCtrl: magic, start_seq, done_seq, test_id, state, result (すべて uint32)
$base = Get-SymbolAddress $Elf "autotune_ctrl"
function Show-Ctrl([uint32[]]$w) {
  Write-Host ("autotune_ctrl @0x{0:X8}: magic=0x{1:X8} start_seq={2} done_seq={3} test_id={4} state={5} result={6}" -f `
      $base, $w[0], $w[1], $w[2], $w[3], $stateNames[[int]$w[4]], $resultNames[[int]$w[5]])
}

$ctrl = Read-Ram32 $base 6
Show-Ctrl $ctrl
if ($Status) { return }

if ($ctrl[4] -ne 0) {
  Write-Error "MainBoard は待機中または走行中です (state=$($stateNames[[int]$ctrl[4]]))。終わってから指示してください"
  exit 1
}

if ($ctrl[2] -eq [uint32]::MaxValue) { $seq = [uint32]0 } else { $seq = [uint32]($ctrl[2] + 1) }
# test_id → magic → start_seq の順に書く (MainBoard は magic と start_seq≠done_seq を見て始める)
Write-Ram32 @(@(($base + 12), $TestId), @($base, $kMagic), @(($base + 4), $seq))

$ctrl = Read-Ram32 $base 6
Show-Ctrl $ctrl
if ($ctrl[0] -ne $kMagic -or $ctrl[3] -ne $TestId -or ($ctrl[1] -ne $seq -and $ctrl[2] -ne $seq)) {
  Write-Error "書いた値を読み戻せませんでした。ELF と書き込み済みの FW が同じか確かめてください"
  exit 1
}
Write-Host ""
Write-Host "開始を指示しました (seq=$seq)。ST-Link を抜いて機体から離れてください。10秒後に走り出します。" -ForegroundColor Yellow
