# ST-Link (SWD, HOTPLUG) で MainBoard の RAM を読み書きする共通の関数。他のスクリプトから dot-source して使う。
#   . (Join-Path $PSScriptRoot "stlink_ram.ps1")
# アドレスはビルドごとに変わるので、毎回 elf から gdb で調べる。
# HOTPLUG なので、つないでもリセットはされない (走行中にはつながないこと。WheelUnit の受信が途切れやすい)

$script:StlinkGdb = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe"
$script:StlinkCli = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
$script:StlinkDefaultElf = Join-Path (Split-Path $PSScriptRoot -Parent) "build\MainBoard_V26_2.elf"

# gdb の式を評価し、出力の最初の 0x... (アドレス) か数値を返す
function Get-GdbValue([string]$Elf, [string]$Expr) {
  $out = & $script:StlinkGdb -batch -ex "print $Expr" $Elf 2>&1 | Out-String
  if ($out -match '0x[0-9a-fA-F]+') { return [Convert]::ToUInt32($Matches[0].Substring(2), 16) }
  if ($out -match '=\s*(\d+)') { return [uint32]$Matches[1] }
  throw "gdb で '$Expr' を評価できませんでした: $out"
}

function Get-SymbolAddress([string]$Elf, [string]$Symbol) {
  return Get-GdbValue $Elf "&$Symbol"
}

function Invoke-StlinkCli([string[]]$CliArgs) {
  $allArgs = @("-c", "port=SWD", "mode=HOTPLUG") + $CliArgs
  $out = & $script:StlinkCli @allArgs 2>&1 | Out-String
  if ($LASTEXITCODE -ne 0 -or $out -match 'Error') {
    throw "STM32_Programmer_CLI が失敗しました (ST-Link はつながっていますか?):`n$out"
  }
  return $out
}

# 4byte 境界の $Address から $Count 個の32bit値を読む
function Read-Ram32([uint32]$Address, [int]$Count) {
  $out = Invoke-StlinkCli @("-r32", ("0x{0:X8}" -f $Address), ($Count * 4))
  $words = New-Object System.Collections.Generic.List[uint32]
  foreach ($line in ($out -split "`r?`n")) {
    if ($line -match '^\s*0x[0-9A-Fa-f]{8}\s*:\s*(.+)$') {
      foreach ($w in ($Matches[1].Trim() -split '\s+')) {
        if ($w -match '^[0-9A-Fa-f]{8}$') { $words.Add([Convert]::ToUInt32($w, 16)) }
      }
    }
  }
  if ($words.Count -lt $Count) { throw "読めた値が足りません ($($words.Count)/$Count):`n$out" }
  return ,($words.GetRange(0, $Count).ToArray())
}

# 16bit 値を読む (4byte 境界でなくてもよい。リトルエンディアン)
function Read-Ram16([uint32]$Address) {
  $aligned = $Address - ($Address % 4)
  $word = (Read-Ram32 $aligned 1)[0]
  $shift = [int](($Address - $aligned) * 8)
  return [uint16](($word -shr $shift) -band 0xFFFF)
}

# $Address から $Size byte を読んでバイト配列で返す
function Read-RamBytes([uint32]$Address, [int]$Size) {
  $tmp = [System.IO.Path]::GetTempFileName()
  try {
    Invoke-StlinkCli @("-u", ("0x{0:X8}" -f $Address), $Size, $tmp) | Out-Null
    $bytes = [System.IO.File]::ReadAllBytes($tmp)
    if ($bytes.Length -lt $Size) { throw "読めたデータが足りません ($($bytes.Length)/$Size byte)" }
    return ,$bytes
  } finally {
    Remove-Item $tmp -ErrorAction SilentlyContinue
  }
}

# (アドレス, 値) の組を順に書く。1回の接続で、並べた順に書かれる
function Write-Ram32([object[]]$Pairs) {
  $cliArgs = @()
  foreach ($p in $Pairs) {
    $cliArgs += @("-w32", ("0x{0:X8}" -f [uint32]$p[0]), ("0x{0:X8}" -f [uint32]$p[1]))
  }
  Invoke-StlinkCli $cliArgs | Out-Null
}
