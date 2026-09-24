# ST-Link から MainBoard に「テストを1回走らせる」指示を書く (src/control/auto_tune.h)。
# 使い方 (ST-Link をつなぎ、機体は床に置いて電源を入れておく。Rock5A からの指令が無い状態):
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1            # 動作パターンを1回
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -TestId 2  # ランプ試験 (向きごとの限界、結果は read_ramp_results.ps1)
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -Status    # 状態を見るだけ
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -Optimize -Rect 3.5,2.5            # 自動最適化 (FF 係数)。合格したらフラッシュに保存。結果は read_opt_results.ps1
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -Optimize -Rect 3.5,2.5 -NoSave    # 保存しない (値は結果に残るだけ)
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -BeepTest                          # ブザーの確認 (走らない。10秒後に鳴る)
#   powershell -ExecutionPolicy Bypass -File tools\autotune_start.ps1 -ClearSaved                        # 保存された調整値を消す (走らない。すぐ消える)
# 書いたら ST-Link を抜いて機体から離れる。10秒後 (LED0 が速く点滅している間) に走り出す。
# 走り終えたら、ST-Link をつないで tools\read_tcs_log.ps1 で記録を読む。
param(
  [int]$TestId = 1,        # 1: 動作パターンのテスト、2: ランプ試験 (AUTOTUNE_TEST_*)
  # この1回だけの volt_tune の上書き (省略 = 既定値。範囲は機体側で収める。走り終わると既定値に戻る)
  [double]$TractionV = 0,      # トルク上限 [V] 例: 2.4
  [double]$MaxAccel = 0,       # S字の加速度上限 [m/s^2]
  [double]$MaxAngAccel = 0,    # S字の角加速度上限 [rad/s^2]
  [double]$KaLat = 0,          # 左右の FF 係数 ka_lat [V/(m/s^2)] (既定 0.5)。FF の較正を、再書き込みなしで試す
  # ランプ試験 (-TestId 2) の速度の段。例: -Speeds 0,1,1.5,2 (0: 止まった状態から、1: 前後左右 1.0m/s、1.5: 斜め、2: 前後左右 2.0m/s、
  #  2d: 斜め 2.0m/s、3: 前後左右 3.0m/s、b2: 前後 2.0m/s のブレーキのみ、b2.5: 前後 2.5m/s、b2l: 左右 2.0m/s (3.5x2.5m のエリアでは走らせられない)、ff: FF 試験 (PI を切って FF だけで加速し、指令と実際の加速度の比を測る。結果は read_ff_results.ps1))。省略 = 0 だけ (従来どおり)。3 は左右を先に走らせてから。空きは前3.5m・後ろ1.0m・左右2.5m
  [string[]]$Speeds = @(),
  # 速度別の測定の範囲 [m] (原点=スタート位置。省略 = 前3.5・後ろ1.0・左右2.5)。例: -XMin -0.5 -XMax 2.0 -YAbs 1.5
  [double]$XMin = 0,     # 後ろ (負の値)
  [double]$XMax = 0,     # 前
  [double]$YAbs = 0,     # 左右 (片側)
  # 長方形のエリア [m] (長辺, 短辺)。例: -Rect 3.5,2.5。機体の中心を「エリアのど真ん中 (対角線の交点)」に、前を長辺に沿って置く
  # (範囲は、各端から 0.2m 内側: XMin=-(長辺/2-0.2), XMax=長辺/2-0.2, YAbs=短辺/2-0.2 を自動で入れる。-XMin/-XMax/-YAbs で上書きできる)
  [string[]]$Rect = @(),   # powershell -File では "3.5,2.5" が1つの文字列になるので、文字列で受けてカンマで分ける
  # 速度別の測定を1回終えたら、機体が自分で範囲の真ん中へ移動し、右へ 90° 回って、範囲の縦横を入れ替えてもう一度繰り返す
  [switch]$Rotate,
  # 自動最適化 (optimizer.h)。-Rect などで範囲を指定する (FF_FAST は範囲の中心から前後 約1m、左右 約1.3m)
  [switch]$Optimize,
  [switch]$NoSave,             # 合格しても保存しない
  [string]$Ref = "rel",        # 比の基準: rel (既定。前後は車輪、左右は IMU の左右/前後) / imu (IMU の絶対値) / wheel (車輪だけ)
  [switch]$BeepTest,           # ブザーの確認 (test_id 6)
  [switch]$ClearSaved,         # 保存された調整値を消す (test_id 5)
  [string]$Elf,
  [switch]$Status
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "stlink_ram.ps1")
if (-not $Elf) { $Elf = $script:StlinkDefaultElf }

$kMagic = [uint32]0x41545331  # "ATS1" (AUTOTUNE_CTRL_MAGIC)
$stateNames = @{ 0 = "IDLE"; 1 = "WAITING"; 2 = "RUNNING" }
$resultNames = @{ 0 = "NONE"; 1 = "FINISHED"; 2 = "ABORTED (安全停止)"; 3 = "CANCELLED (Rock5A)"; 4 = "BAD_TEST" }
if ($Optimize) { $TestId = 4 } elseif ($BeepTest) { $TestId = 6 } elseif ($ClearSaved) { $TestId = 5 }
[uint32]$optFlags = 0
if ($Optimize -and -not $NoSave) { $optFlags = $optFlags -bor 1 }
if ($Ref.ToLower() -eq "imu") { $optFlags = $optFlags -bor 2 } elseif ($Ref.ToLower() -eq "wheel") { $optFlags = $optFlags -bor 4 } elseif ($Ref.ToLower() -ne "rel") { Write-Error "-Ref は rel, imu, wheel のどれか"; exit 1 }

# AutoTuneCtrl: magic, start_seq, done_seq, test_id, state, result, traction_x100, max_accel_x100, max_ang_accel_x100, ramp_speed_mask, ramp_x_min_cm, ramp_x_max_cm, ramp_y_abs_cm, ka_lat_x1000, opt_task_mask, opt_flags (すべて 32bit。範囲は cm の符号付き)
$base = Get-SymbolAddress $Elf "autotune_ctrl"
function Show-Ctrl([uint32[]]$w) {
  Write-Host ("autotune_ctrl @0x{0:X8}: magic=0x{1:X8} start_seq={2} done_seq={3} test_id={4} state={5} result={6} override(x100)=[traction {7}, accel {8}, ang_accel {9}] speed_mask=0x{10:X2} area(cm)=x[{11},{12}] y+-{13} ka_lat_x1000={14}" -f `
      $base, $w[0], $w[1], $w[2], $w[3], $stateNames[[int]$w[4]], $resultNames[[int]$w[5]], $w[6], $w[7], $w[8], $w[9], [BitConverter]::ToInt32([BitConverter]::GetBytes([uint32]$w[10]), 0), [BitConverter]::ToInt32([BitConverter]::GetBytes([uint32]$w[11]), 0), [BitConverter]::ToInt32([BitConverter]::GetBytes([uint32]$w[12]), 0), $w[13])
}

$ctrl = Read-Ram32 $base 16
Show-Ctrl $ctrl
if ($Status) { return }

# -Rect から範囲を求める (明示した -XMin/-XMax/-YAbs があればそちらを優先)
$rectValues = @($Rect | ForEach-Object { $_ -split "," } | Where-Object { $_.Trim() -ne "" } | ForEach-Object { [double]$_.Trim() })
if ($rectValues.Count -eq 2) {
  $long = $rectValues[0]; $short = $rectValues[1]
  if ($XMin -eq 0) { $XMin = -[math]::Round($long / 2 - 0.2, 2) }
  if ($XMax -eq 0) { $XMax = [math]::Round($long / 2 - 0.2, 2) }
  if ($YAbs -eq 0) { $YAbs = [math]::Round($short / 2 - 0.2, 2) }
} elseif ($rectValues.Count -ne 0) { Write-Error "-Rect は 長辺,短辺 の2つの数 [m] (例: -Rect 3.5,2.5)"; exit 1 }

# 速度の段 → ビットの組み合わせ (ramp_test.h の RAMP_SPEED_*)
$speedBits = @{ "0" = 0x01; "1" = 0x02; "1.5" = 0x04; "2" = 0x08; "3" = 0x10; "2d" = 0x20; "b2" = 0x40; "b2.5" = 0x100; "b2l" = 0x200; "ff" = 0x400; "fast" = 0x1000 }
[uint32]$speedMask = 0
foreach ($sp in ($Speeds | ForEach-Object { $_ -split "," } | Where-Object { $_ -ne "" })) {
  $key = $sp.Trim().ToLower().Replace("1.0", "1").Replace("2.0", "2").Replace("3.0", "3")
  if (-not $speedBits.ContainsKey($key)) { Write-Error "-Speeds に使えない値: $sp (0, 1, 1.5, 2, 2d, 3, b2, b2.5, b2l, ff, fast)"; exit 1 }
  $speedMask = $speedMask -bor [uint32]$speedBits[$key]
}
if ($Rotate) { $speedMask = $speedMask -bor 0x80 }
if ($TestId -ne 2 -and $speedMask -ne 0) { Write-Error "-Speeds は -TestId 2 (ランプ試験) のときだけ使えます"; exit 1 }
if ($Optimize) {
  if ($XMax -eq 0) { Write-Error "-Optimize には範囲が要ります (例: -Rect 3.5,2.5)"; exit 1 }
  Write-Host ("自動最適化: 範囲 x[{0},{1}] y±{2} m。機体は範囲の中心に、前を長辺に沿って置く。約 4〜8 回、6〜12 分走る。{3}" -f $XMin, $XMax, $YAbs, $(if ($optFlags -band 1) { "合格したらフラッシュに保存します (試合でも使われる)" } else { "保存しません" })) -ForegroundColor Yellow
}
if ($speedMask -band 0x7FE) {
  if ($XMax -ne 0) {
    Write-Host ("速度別の測定: 範囲 x[{0},{1}] y±{2} m (mask=0x{3}{4})。この範囲に何も無いことを確かめてください" -f $XMin, $XMax, $YAbs, $speedMask.ToString("X2"), $(if ($Rotate) { "、1回目のあと機体が90°回って繰り返す" } else { "" })) -ForegroundColor Yellow
  } else {
    Write-Host ("速度別の測定: 前3.5m・後ろ1.0m・左右2.5m の空きが要ります (mask=0x{0})" -f $speedMask.ToString("X2")) -ForegroundColor Yellow
  }
}

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
  @(($base + 36), $speedMask),
  @(($base + 40), [BitConverter]::ToUInt32([BitConverter]::GetBytes([int32][math]::Round($XMin * 100)), 0)),
  @(($base + 44), [BitConverter]::ToUInt32([BitConverter]::GetBytes([int32][math]::Round($XMax * 100)), 0)),
  @(($base + 48), [BitConverter]::ToUInt32([BitConverter]::GetBytes([int32][math]::Round($YAbs * 100)), 0)),
  @(($base + 52), [uint32][math]::Round($KaLat * 1000)),
  @(($base + 56), [uint32]1),
  @(($base + 60), $optFlags),
  @($base, $kMagic), @(($base + 4), $seq))
Write-Ram32 $writes

$ctrl = Read-Ram32 $base 16
Show-Ctrl $ctrl
if ($ctrl[0] -ne $kMagic -or $ctrl[3] -ne $TestId -or ($ctrl[1] -ne $seq -and $ctrl[2] -ne $seq)) {
  Write-Error "書いた値を読み戻せませんでした。ELF と書き込み済みの FW が同じか確かめてください"
  exit 1
}
Write-Host ""
if ($TestId -eq 5) { Write-Host "保存された調整値を消すよう指示しました。すぐ消えます (Status で done_seq を確認)。" -ForegroundColor Yellow }
elseif ($TestId -eq 6) { Write-Host "ブザーの確認を指示しました (seq=$seq)。10秒後に上がる3音が鳴ります (走りません)。" -ForegroundColor Yellow }
else { Write-Host "開始を指示しました (seq=$seq)。ST-Link を抜いて機体から離れてください。10秒後に走り出します。" -ForegroundColor Yellow }
