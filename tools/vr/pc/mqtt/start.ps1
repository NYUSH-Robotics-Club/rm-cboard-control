param([switch]$NeutralOnly, [switch]$NoWindow)
$ErrorActionPreference = 'Stop'
$vrRoot = Split-Path (Split-Path $PSScriptRoot)
$settings = Get-Content -LiteralPath (Join-Path $vrRoot 'config.local.json') -Raw | ConvertFrom-Json
$python = [string]$settings.python_path
if (-not (Test-Path -LiteralPath $python)) { throw 'Configure python_path in tools/vr/config.local.json' }
$created = $false
$instance = [Threading.Mutex]::new($true, 'Local\FrankaBotVehicleMqtt', [ref]$created)
if (-not $created) { $instance.Dispose(); Write-Host 'MQTT listener is already running.'; exit 0 }
$env:PYTHONIOENCODING = 'utf-8'
try {
    if (-not $NoWindow) {
        $pythonw = Join-Path (Split-Path $python) 'pythonw.exe'
        $entry = Join-Path $PSScriptRoot 'status_window.py'
        $windows = Get-CimInstance Win32_Process -Filter "Name='pythonw.exe'" | Where-Object {
            $_.CommandLine -and $_.CommandLine.Contains($entry)
        }
        if (-not $windows) {
            Start-Process -FilePath $pythonw -ArgumentList ('"' + $entry + '"') -WorkingDirectory $vrRoot -WindowStyle Hidden
        }
    }
    $entryArgs = @('-u', (Join-Path $PSScriptRoot 'bridge.py'))
    if ($NeutralOnly) { $entryArgs += '--neutral-only' }
    & $python @entryArgs 2>&1 | Tee-Object -FilePath (Join-Path $vrRoot 'runtime/mqtt-live.log')
} finally { $instance.ReleaseMutex(); $instance.Dispose() }
