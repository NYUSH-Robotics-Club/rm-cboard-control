$ErrorActionPreference = 'Stop'
$entry = Join-Path $PSScriptRoot 'status_window.py'
$running = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -and
    $_.CommandLine.Replace('/', '\').IndexOf($entry.Replace('/', '\'), [StringComparison]::OrdinalIgnoreCase) -ge 0
}
if (-not $running) {
    $python = Join-Path (Split-Path (Split-Path $PSScriptRoot)) '.venv/Scripts/pythonw.exe'
    Start-Process -FilePath $python -ArgumentList ('"' + $entry + '"') -WindowStyle Normal
}
