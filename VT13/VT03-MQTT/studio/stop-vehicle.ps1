$ErrorActionPreference = 'Stop'
Set-Content -LiteralPath (Join-Path $PSScriptRoot 'data\vehicle-stop.request') -Value 'stop' -Encoding ASCII
Write-Host 'Neutral/stop requested; UI listener remains running.'
