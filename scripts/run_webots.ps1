param([switch]$Batch, [string]$RunId = (Get-Date -Format 'yyyyMMdd_HHmmss'))
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$webotsRoot = if ($env:WEBOTS_HOME) { $env:WEBOTS_HOME } else { 'C:\Program Files\Webots' }
$webotsExe = Join-Path $webotsRoot 'msys64\mingw64\bin\webots.exe'
if (-not (Test-Path -LiteralPath $webotsExe)) { throw 'Webots was not found. Set WEBOTS_HOME.' }
$env:SAR_RUN_ID = $RunId
$env:SAR_BATCH = if ($Batch) { '1' } else { '0' }
$env:SAR_CAPTURE = '1'
$env:PYTHONIOENCODING = 'utf-8'
if (-not $env:SAR_PYTHON) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    $env:SAR_PYTHON = if (Test-Path -LiteralPath $bundledPython) { $bundledPython } else { (Get-Command python).Source }
}
$worldPath = Join-Path $projectRoot 'worlds\rescue_medium.wbt'
if ($Batch) {
    & $webotsExe --batch --minimize --mode=fast --stdout --stderr $worldPath
} else {
    & $webotsExe --mode=pause $worldPath
}
