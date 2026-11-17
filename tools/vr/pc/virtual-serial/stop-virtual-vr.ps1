$ErrorActionPreference = 'Stop'
# Restrict termination to this exact adapter, including the venv launcher child.
$entry = Join-Path $PSScriptRoot 'run_virtual_vr.py'
$processes = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -and
    $_.CommandLine.Replace('/', '\').IndexOf($entry.Replace('/', '\'), [StringComparison]::OrdinalIgnoreCase) -ge 0
}
foreach ($process in $processes) {
    Stop-Process -Id $process.ProcessId -ErrorAction SilentlyContinue
}
Write-Output 'VR virtual-serial listener stopped. The stock client and VSPE remain running.'
