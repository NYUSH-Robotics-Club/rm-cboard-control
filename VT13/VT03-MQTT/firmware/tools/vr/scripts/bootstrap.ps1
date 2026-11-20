$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot
if (-not (Test-Path (Join-Path $root '.venv/Scripts/python.exe'))) {
    py -3 -m venv (Join-Path $root '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Python 3 with tkinter is required.' }
}
& (Join-Path $root '.venv/Scripts/python.exe') -m pip install -r (Join-Path $root 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Python dependency installation failed.' }
if (-not (Test-Path (Join-Path $root 'config.local.json'))) {
    Copy-Item (Join-Path $root 'config.example.json') (Join-Path $root 'config.local.json')
}
Write-Output 'Edit config.local.json for the original player path, ADB and optional Quest serial.'
