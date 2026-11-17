param(
    [ValidateSet('Install', 'Alias', 'RemoveAlias')][string]$Step = 'Install',
    [ValidateRange(1, 256)][int]$ClientPort = 22
)
$ErrorActionPreference = 'Stop'
$log = Join-Path $PSScriptRoot ('setup-' + $Step + '.log')
Start-Transcript -Path $log -Force | Out-Null
try {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Administrator elevation is required for driver/device installation.'
    }
    if ($Step -eq 'Install') {
        $msi = Get-ChildItem (Join-Path $PSScriptRoot 'install/vspe') -Recurse -Filter '*.msi' |
            Where-Object { $_.Name -match '64' } | Select-Object -First 1
        if (-not $msi) { throw 'The vendor x64 MSI has not been downloaded.' }
        $signature = Get-AuthenticodeSignature -LiteralPath $msi.FullName
        if ($signature.Status -ne 'Valid') { throw ('Invalid installer signature: ' + $signature.Status) }
        Write-Output ('Verified installer signer: ' + $signature.SignerCertificate.Subject)
        $installLog = Join-Path $PSScriptRoot 'vspe-msi-install.log'
        $arguments = '/i "' + $msi.FullName + '" /qn /norestart /L*v "' + $installLog + '"'
        $process = Start-Process msiexec.exe -ArgumentList $arguments -Wait -PassThru -WindowStyle Hidden
        if ($process.ExitCode -notin @(0, 3010)) { throw ('MSI exit code ' + $process.ExitCode) }
        Write-Output ('INSTALL_OK exit=' + $process.ExitCode)
    } else {
        # This separate root device supplies discovery metadata only. Actual
        # byte transport is provided by VSPE, not by this device or this script.
        Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using System.ComponentModel;
using System.Text;
public static class VrSerialDiscoveryNode {
    [StructLayout(LayoutKind.Sequential)] struct Info {
        public uint cbSize; public Guid ClassGuid; public uint DevInst; public UIntPtr Reserved;
    }
    [DllImport("setupapi.dll", SetLastError=true)] static extern IntPtr SetupDiCreateDeviceInfoList(IntPtr guid, IntPtr hwnd);
    [DllImport("setupapi.dll", CharSet=CharSet.Unicode, SetLastError=true)] static extern bool SetupDiCreateDeviceInfoW(IntPtr set, string name, ref Guid guid, string desc, IntPtr hwnd, uint flags, ref Info info);
    [DllImport("setupapi.dll", CharSet=CharSet.Unicode, SetLastError=true)] static extern bool SetupDiSetDeviceRegistryPropertyW(IntPtr set, ref Info info, uint property, byte[] buffer, uint size);
    [DllImport("setupapi.dll", SetLastError=true)] static extern bool SetupDiCallClassInstaller(uint function, IntPtr set, ref Info info);
    [DllImport("setupapi.dll", CharSet=CharSet.Unicode, SetLastError=true)] static extern bool SetupDiOpenDeviceInfoW(IntPtr set, string instance, IntPtr hwnd, uint flags, ref Info info);
    [DllImport("setupapi.dll", CharSet=CharSet.Unicode, SetLastError=true)] static extern bool SetupDiGetDeviceInstanceIdW(IntPtr set, ref Info info, StringBuilder id, uint size, out uint required);
    [DllImport("setupapi.dll")] static extern bool SetupDiDestroyDeviceInfoList(IntPtr set);
    static void Check(bool ok) { if (!ok) throw new Win32Exception(Marshal.GetLastWin32Error()); }
    static void Set(IntPtr set, ref Info info, uint property, string value) {
        byte[] bytes = Encoding.Unicode.GetBytes(value); Check(SetupDiSetDeviceRegistryPropertyW(set, ref info, property, bytes, (uint)bytes.Length));
    }
    public static string Create(int port) {
        IntPtr set = SetupDiCreateDeviceInfoList(IntPtr.Zero, IntPtr.Zero);
        if (set == new IntPtr(-1)) throw new Win32Exception(Marshal.GetLastWin32Error());
        try {
            Info info = new Info(); info.cbSize = (uint)Marshal.SizeOf(typeof(Info));
            Guid system = new Guid("4d36e97d-e325-11ce-bfc1-08002be10318");
            Check(SetupDiCreateDeviceInfoW(set, "VRFTDI_DISCOVERY", ref system,
                "VR virtual serial discovery metadata", IntPtr.Zero, 1, ref info));
            Set(set, ref info, 1, "VRFTDI\\CUSTOMCONTROLLER\0\0");
            Set(set, ref info, 12, "VR Virtual Serial Discovery (COM" + port + ")\0");
            Check(SetupDiCallClassInstaller(0x19, set, ref info));
            StringBuilder id = new StringBuilder(512); uint required;
            Check(SetupDiGetDeviceInstanceIdW(set, ref info, id, 512, out required));
            return id.ToString();
        } finally { SetupDiDestroyDeviceInfoList(set); }
    }
    public static void Remove(string instance) {
        if (!instance.StartsWith("ROOT\\VRFTDI_DISCOVERY\\", StringComparison.OrdinalIgnoreCase))
            throw new ArgumentException("Refusing to remove a non-VR device");
        IntPtr set = SetupDiCreateDeviceInfoList(IntPtr.Zero, IntPtr.Zero);
        if (set == new IntPtr(-1)) throw new Win32Exception(Marshal.GetLastWin32Error());
        try {
            Info info = new Info(); info.cbSize = (uint)Marshal.SizeOf(typeof(Info));
            Check(SetupDiOpenDeviceInfoW(set, instance, IntPtr.Zero, 0, ref info));
            Check(SetupDiCallClassInstaller(5, set, ref info));
        } finally { SetupDiDestroyDeviceInfoList(set); }
    }
}
'@
        $statePath = Join-Path $PSScriptRoot 'discovery-node.json'
        if ($Step -eq 'Alias') {
            $portAvailable = [IO.Ports.SerialPort]::GetPortNames() -contains ('COM' + $ClientPort)
            if (-not $portAvailable) { throw ('Create the VSPE pair first; COM' + $ClientPort + ' does not exist.') }
            $existing = @(Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -like 'ROOT\VRFTDI_DISCOVERY\*' })
            if ($existing.Count) { throw 'A VR discovery node already exists. Inspect/remove it before creating another.' }
            $node = [VrSerialDiscoveryNode]::Create($ClientPort)
            @{ instance_id = $node; client_port = $ClientPort; purpose = 'Discovery metadata only; VSPE transports bytes' } |
                ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding UTF8
            Write-Output ('ALIAS_OK ' + $node)
        } else {
            $state = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
            [VrSerialDiscoveryNode]::Remove($state.instance_id)
            Move-Item -LiteralPath $statePath -Destination ($statePath + '.removed-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
            Write-Output 'ALIAS_REMOVED'
        }
    }
} catch {
    Write-Output ('SETUP_FAILED ' + $_.Exception.Message)
    throw
} finally {
    Stop-Transcript | Out-Null
}
