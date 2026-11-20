$Host.UI.RawUI.WindowTitle='Quest calibration - Ctrl+C to finish'
$env:PYTHONIOENCODING='utf-8'
$p=Join-Path (Split-Path $PSScriptRoot) '.venv/Scripts/python.exe'
& $p -u (Join-Path $PSScriptRoot 'start-existing-quest-calibration.py') 2>&1 | Tee-Object -FilePath (Join-Path $PSScriptRoot 'car-calibration.log')
