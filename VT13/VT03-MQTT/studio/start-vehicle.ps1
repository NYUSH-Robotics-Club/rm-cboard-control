$ErrorActionPreference = 'Stop'
$studioRoot = $PSScriptRoot
$boardRecordPath = Join-Path $studioRoot 'data\board-ready.json'
if (-not (Test-Path -LiteralPath $boardRecordPath)) { throw 'Dual-controller firmware has not been verified on the C board. Deploy and verify it first; old driving remains available.' }
$boardRecord = Get-Content -LiteralPath $boardRecordPath -Raw | ConvertFrom-Json
if (-not $boardRecord.flash_verified -or $boardRecord.dual_tag -ne '56524433') { throw 'Dual-controller board verification record is invalid.' }
$legacyRoot = Join-Path (Split-Path $studioRoot) 'firmware\tools\vr'
$status = Invoke-RestMethod 'http://127.0.0.1:18780/api/status' -TimeoutSec 2
if ($status.publisher.running) { Write-Host 'Dual-controller publisher already running.'; exit }
# Controlled switch of publisher ownership. No original source or config edits.
& (Join-Path $legacyRoot 'pc\mqtt\stop.ps1')
$legacyStopped = $false
for ($attempt = 0; $attempt -lt 40; $attempt++) {
    if (-not (Test-Path (Join-Path $legacyRoot 'runtime\mqtt-status.json'))) { $legacyStopped = $true; break }
    $legacyState = Get-Content -LiteralPath (Join-Path $legacyRoot 'runtime\mqtt-status.json') -Raw | ConvertFrom-Json
    if (-not $legacyState.listener_running) { $legacyStopped = $true; break }
    Start-Sleep -Milliseconds 100
}
if (-not $legacyStopped) { throw 'Old publisher has not stopped; no second publisher was started.' }
$pythonExe = Join-Path $studioRoot '.venv\Scripts\python.exe'
$quotedScript = '"' + (Join-Path $studioRoot 'mqtt_vehicle.py') + '"'
Start-Process -FilePath $pythonExe -ArgumentList $quotedScript -WorkingDirectory $studioRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $studioRoot 'data\vehicle.stdout.log') -RedirectStandardError (Join-Path $studioRoot 'data\vehicle.stderr.log')
Write-Host 'Publisher started in neutral. Open editor; enter VR and release both triggers before arming.'
