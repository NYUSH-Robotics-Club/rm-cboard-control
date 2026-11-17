$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot
$folder = Join-Path $root 'pc/virtual-serial/install'
New-Item -ItemType Directory -Path $folder -Force | Out-Null
$archive = Join-Path $folder 'SetupVSPE_64_1.5.8.489.zip'
Invoke-WebRequest -Uri 'https://eterlogic.com/downloads/SetupVSPE_64_1.5.8.489.zip' -OutFile $archive
Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $folder 'vspe') -Force
$msi = Get-ChildItem (Join-Path $folder 'vspe') -Recurse -Filter '*64*.msi' | Select-Object -First 1
$signature = Get-AuthenticodeSignature -LiteralPath $msi.FullName
if ($signature.Status -ne 'Valid') { throw 'Installer signature validation failed.' }
Write-Output ('Verified signer: ' + $signature.SignerCertificate.Subject)
Write-Output 'Installer is ready. Run pc/virtual-serial/setup-virtual-serial.ps1 -Step Install as administrator.'
