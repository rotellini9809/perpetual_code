param([switch]$SkipInstall)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRoot = Join-Path $projectRoot '.pio'
$desktopPython = Join-Path $buildRoot 'perpetual_app_venv\Scripts\python.exe'
if (!(Test-Path -LiteralPath $desktopPython)) {
    & python -m venv (Join-Path $buildRoot 'perpetual_app_venv')
    if ($LASTEXITCODE -ne 0) { throw 'Creazione ambiente desktop fallita' }
}
if (!$SkipInstall) {
    & $desktopPython -m pip install --disable-pip-version-check -r (Join-Path $PSScriptRoot 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Installazione dipendenze desktop fallita' }
}
& $desktopPython -m PyInstaller --noconfirm --distpath (Join-Path $buildRoot 'desktop_dist') --workpath (Join-Path $buildRoot 'desktop_build') (Join-Path $PSScriptRoot 'Perpetual.spec')
if ($LASTEXITCODE -ne 0) { throw 'Build desktop fallita' }
Write-Output (Join-Path $buildRoot 'desktop_dist\Perpetual\Perpetual.exe')
