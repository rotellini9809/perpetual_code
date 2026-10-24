param(
    [string]$InputCsv,
    [string]$OutputCsv
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
if (!$InputCsv) {
    $InputCsv = Join-Path $projectRoot 'datasets/hoppino_VOLO_UFFICIALE.csv'
}
$InputCsv = (Resolve-Path -LiteralPath $InputCsv).Path
$buildDir = Join-Path $projectRoot '.pio'
New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
if (!$OutputCsv) {
    $OutputCsv = Join-Path $buildDir ('hoppino_events_' + [guid]::NewGuid().ToString('N') + '.csv')
}
$exePath = Join-Path $buildDir 'hoppino_replay.exe'
& g++ -std=c++11 -Wall -Wextra -Werror -pedantic "-I$(Join-Path $projectRoot 'include')" `
    (Join-Path $projectRoot 'src/configuration/RecoveryConfiguration.cpp') `
    (Join-Path $projectRoot 'src/configuration/RecoveryRuleEngine.cpp') `
    (Join-Path $PSScriptRoot 'hoppino_replay.cpp') -o $exePath
if ($LASTEXITCODE -ne 0) { throw 'Replay compilation failed' }
& $exePath $InputCsv $OutputCsv
if ($LASTEXITCODE -ne 0) { throw 'Replay failed' }
