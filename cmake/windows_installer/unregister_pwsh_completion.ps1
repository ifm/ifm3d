#Requires -Version 5.1
<#
.SYNOPSIS
    Removes the ifm3d PowerShell tab completion block from the PowerShell profile.

.DESCRIPTION
    Strips the guarded block that was written by register_pwsh_completion.ps1.
    Running on a profile that was never registered is a safe no-op.

.PARAMETER AllUsers
    When specified, targets the AllUsersAllHosts profile (used by the NSIS
    uninstaller, which runs elevated).  Without this switch the current user's
    CurrentUserAllHosts profile is cleaned.
#>
param(
    [switch]$AllUsers
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ($AllUsers) {
    $principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltinRole]::Administrator)) {
        throw "AllUsers unregistration requires Administrator privileges."
    }
}

# Returns AllUsers profile paths for both 64-bit (System32) and 32-bit (SysWOW64)
# Windows PowerShell. Sysnative bypasses the WOW64 redirector from 32-bit processes.
function Get-AllUsersProfilePaths {
    if ([System.Environment]::Is64BitProcess) {
        $nativeDir = [System.IO.Path]::Combine($env:windir, 'System32',  'WindowsPowerShell', 'v1.0')
    } else {
        $nativeDir = [System.IO.Path]::Combine($env:windir, 'Sysnative', 'WindowsPowerShell', 'v1.0')
    }
    $wow64Dir = [System.IO.Path]::Combine($env:windir, 'SysWOW64', 'WindowsPowerShell', 'v1.0')

    $paths = @()
    if (Test-Path $nativeDir) { $paths += [System.IO.Path]::Combine($nativeDir, 'profile.ps1') }
    if (Test-Path $wow64Dir)  { $paths += [System.IO.Path]::Combine($wow64Dir,  'profile.ps1') }
    return $paths
}

if ($AllUsers) {
    $profilePaths = Get-AllUsersProfilePaths
} else {
    $profilePaths = @($PROFILE.CurrentUserAllHosts)
}

$utf8NoBom = [System.Text.UTF8Encoding]::new($false)

$removedAnyBlock = $false

$pattern = '(?ms)^\s*# >>> ifm3d tab completion begin\r?\n.*?\r?\n# <<< ifm3d tab completion end\s*'

foreach ($profilePath in $profilePaths) {
    if (-not (Test-Path -LiteralPath $profilePath)) {
        Write-Host "ifm3d: no PowerShell profile found; nothing to remove."
        Write-Host "  $profilePath"
        continue
    }

    $content = Get-Content -LiteralPath $profilePath -Raw -ErrorAction SilentlyContinue
    if ([string]::IsNullOrEmpty($content)) { continue }

    $replaced = [regex]::Replace($content, $pattern, '')
    if ($replaced -eq $content) {
        Write-Host "ifm3d: no managed completion block found; nothing to remove."
        Write-Host "  $profilePath"
        continue
    }

    $newContent = $replaced.TrimEnd("`r", "`n")
    if (-not [string]::IsNullOrEmpty($newContent)) {
        $newContent += "`r`n"
    }

    [System.IO.File]::WriteAllText($profilePath, $newContent, $utf8NoBom)
    $removedAnyBlock = $true
    Write-Host "ifm3d: PowerShell tab completion removed from:"
    Write-Host "  $profilePath"
}

if (-not $AllUsers -and $removedAnyBlock -and $profilePaths.Count -gt 0) {
    $profileDir = Split-Path $profilePaths[0] -Parent
    $perUserCompletionScript = Join-Path $profileDir 'ifm3d_completion.ps1'
    if (Test-Path -LiteralPath $perUserCompletionScript) {
        Remove-Item -LiteralPath $perUserCompletionScript -Force
        Write-Host "ifm3d: removed completion script copy:"
        Write-Host "  $perUserCompletionScript"
    }
}
