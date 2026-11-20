$ErrorActionPreference = 'Stop'
$vrRoot = Split-Path (Split-Path $PSScriptRoot)
$runtime = Join-Path $vrRoot 'runtime'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
Set-Content -LiteralPath (Join-Path $runtime 'mqtt-stop.request') -Value 'stop' -Encoding ASCII
Write-Host 'Stop requested. The listener will send zero and exit; official client remains running.'
