param([switch]$SkipInstall, [switch]$Zip)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRoot = Join-Path $projectRoot '.pio'
$desktopVenv = Join-Path $projectRoot '.venv'
$desktopPython = Join-Path $desktopVenv 'Scripts\python.exe'
if (!(Test-Path -LiteralPath $desktopPython)) {
    & python -m venv $desktopVenv
    if ($LASTEXITCODE -ne 0) { throw 'Desktop environment creation failed' }
}
if (!$SkipInstall) {
    & $desktopPython -m pip install --disable-pip-version-check -r (Join-Path $PSScriptRoot 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Desktop dependency installation failed' }
}
& $desktopPython -m PyInstaller --noconfirm --distpath (Join-Path $buildRoot 'desktop_dist') --workpath (Join-Path $buildRoot 'desktop_build') (Join-Path $PSScriptRoot 'Perpetual.spec')
if ($LASTEXITCODE -ne 0) { throw 'Desktop build failed' }
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'PORTABLE.txt') -Destination (Join-Path $buildRoot 'desktop_dist\Perpetual\README.txt') -Force
Write-Output (Join-Path $buildRoot 'desktop_dist\Perpetual\Perpetual.exe')
if ($Zip) {
    $zipPath = Join-Path $buildRoot 'desktop_dist\Perpetual-windows-x64.zip'
    Compress-Archive -Path (Join-Path $buildRoot 'desktop_dist\Perpetual') -DestinationPath $zipPath -Force
    Write-Output $zipPath
}
