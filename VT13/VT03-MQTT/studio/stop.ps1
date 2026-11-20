$ErrorActionPreference = 'Stop'
$recordPath = Join-Path $PSScriptRoot 'data\process.json'
if (-not (Test-Path -LiteralPath $recordPath)) { Write-Host 'No process record for this studio.'; exit }
$record = Get-Content -LiteralPath $recordPath -Raw | ConvertFrom-Json
$expectedScript = Join-Path $PSScriptRoot 'server.py'
if ($record.script -ne $expectedScript) { throw 'Process record does not refer to this studio.' }
$processInfo = Get-CimInstance Win32_Process -Filter ('ProcessId = {0}' -f [int]$record.pid)
if (-not $processInfo) { Write-Host 'Studio is already stopped.'; exit }
if ($processInfo.Name -ne 'python.exe' -or -not $processInfo.CommandLine.Contains(('"' + $expectedScript + '"')) -or -not $processInfo.CommandLine.Contains('--port 18780')) { throw 'PID belongs to a different command; no process was stopped.' }
$listenerInfo = Get-NetTCPConnection -LocalPort 18780 -State Listen -ErrorAction SilentlyContinue
if ($listenerInfo -and $listenerInfo.OwningProcess -ne [int]$record.pid) {
    $workerInfo = Get-CimInstance Win32_Process -Filter ('ProcessId = {0}' -f [int]$listenerInfo.OwningProcess)
    if ($workerInfo.Name -ne 'python.exe' -or -not $workerInfo.CommandLine.Contains(('"' + $expectedScript + '"')) -or -not $workerInfo.CommandLine.Contains('--port 18780')) { throw 'Listener belongs to a different command; no process was stopped.' }
    Stop-Process -Id ([int]$listenerInfo.OwningProcess)
}
Stop-Process -Id ([int]$record.pid) -ErrorAction SilentlyContinue
Write-Host 'Only the independent UI studio was stopped. Existing VR/MQTT applications were not changed.'
