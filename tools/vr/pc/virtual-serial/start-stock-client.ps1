$ErrorActionPreference = 'Stop'
$python = Join-Path (Split-Path (Split-Path $PSScriptRoot)) '.venv/Scripts/python.exe'
& $python (Join-Path $PSScriptRoot 'check_stock_hashes.py')
if ($LASTEXITCODE -ne 0) { throw 'Original client binary verification failed.' }
$vrRoot = Split-Path (Split-Path $PSScriptRoot)
$configPath = Join-Path $vrRoot 'config.local.json'
if (-not (Test-Path -LiteralPath $configPath)) { $configPath = Join-Path $vrRoot 'config.example.json' }
$config = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
& $python (Join-Path $PSScriptRoot 'discovery.py') --require-controller $config.player_port
if ($LASTEXITCODE -ne 0) { throw 'Original client does not discover virtual COM22.' }
$folder = Split-Path $config.stock_client_root
Start-Process -FilePath (Join-Path $folder 'start.bat') -WorkingDirectory $folder -WindowStyle Hidden
