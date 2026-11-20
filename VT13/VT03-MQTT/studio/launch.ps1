param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$studioRoot = $PSScriptRoot
$pythonExe = Join-Path $studioRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) { throw 'Create .venv and install requirements.txt first.' }
if (-not (Test-Path -LiteralPath $pythonExe)) { throw 'Python 3 was not found; run python server.py manually.' }
try {
    $bootstrap = Invoke-RestMethod 'http://127.0.0.1:18780/api/bootstrap' -TimeoutSec 2
    if ($bootstrap.mode -ne 'dual-vr-studio') { throw 'Port 18780 is occupied by another service or an earlier build; stop only the studio before restarting.' }
} catch {
    $busy = Get-NetTCPConnection -LocalPort 18780 -State Listen -ErrorAction SilentlyContinue
    if ($busy) { throw 'Port 18780 is occupied; no existing process was changed.' }
    $quotedScript = '"' + (Join-Path $studioRoot 'server.py') + '"'
    $studioProcess = Start-Process -FilePath $pythonExe -ArgumentList @($quotedScript, '--port', '18780', '--video') -WorkingDirectory $studioRoot -WindowStyle Hidden -RedirectStandardOutput (Join-Path $studioRoot 'data\server.stdout.log') -RedirectStandardError (Join-Path $studioRoot 'data\server.stderr.log') -PassThru
    @{ pid = $studioProcess.Id; script = (Join-Path $studioRoot 'server.py'); started = (Get-Date).ToString('o') } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $studioRoot 'data\process.json') -Encoding UTF8
    $ready = $false
    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        Start-Sleep -Milliseconds 150
        try { $bootstrap = Invoke-RestMethod 'http://127.0.0.1:18780/api/bootstrap' -TimeoutSec 1; $ready = ($bootstrap.mode -eq 'dual-vr-studio'); if ($ready) { break } } catch {}
    }
    if (-not $ready) { throw 'Studio did not start; check data/server.stderr.log.' }
    # Windows venv python may use a launcher parent; record the HTTP worker PID.
    @{ pid = $bootstrap.server_pid; script = (Join-Path $studioRoot 'server.py'); started = (Get-Date).ToString('o') } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $studioRoot 'data\process.json') -Encoding UTF8
}
if (-not $NoBrowser) { Start-Process 'http://127.0.0.1:18780/' }
# Deliberately no ADB, APK, MQTT, serial, SSH, or existing application operations.
