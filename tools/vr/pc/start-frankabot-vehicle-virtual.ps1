param([string]$Serial = '')
$ErrorActionPreference = 'Stop'
$env:PYTHONIOENCODING = 'utf-8'
$root = Split-Path $PSScriptRoot
$python = Join-Path $root '.venv/Scripts/python.exe'
& (Join-Path $PSScriptRoot 'virtual-serial/show-vr-status.ps1')
& $python (Join-Path $PSScriptRoot 'virtual-serial/check_stock_hashes.py')
if ($LASTEXITCODE -ne 0) { throw 'Original player binary verification failed.' }
$arguments = @('-u', (Join-Path $PSScriptRoot 'virtual-serial/run_virtual_vr.py'))
if ($Serial) { $arguments += @('--serial', $Serial) }
New-Item -ItemType Directory -Path (Join-Path $root 'runtime') -Force | Out-Null
& $python @arguments 2>&1 | Tee-Object -FilePath (Join-Path $root 'runtime/vr-live.log')
