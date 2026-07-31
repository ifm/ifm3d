#Requires -Version 5.1
<#
.SYNOPSIS
    Registers ifm3d PowerShell tab completion in the appropriate PowerShell profile.

.DESCRIPTION
    Appends a guarded block to the PowerShell profile that dot-sources the ifm3d
    completion script.  Running a second time is idempotent.

    The NSIS installer calls this with -AllUsers so the machine-wide profiles are
    modified and completion works for every user without requiring per-user action.
    Both the 64-bit (System32) and 32-bit (SysWOW64) AllUsers profiles are updated
    so that both flavours of Windows PowerShell get tab completion.

    For a manual / build-from-source install, call without -AllUsers to write only
    to the current user's profile (CurrentUserAllHosts).

.PARAMETER InstallDir
    The directory where ifm3d was installed, e.g. C:\Program Files\ifm3d

.PARAMETER AllUsers
    When specified, modifies the AllUsersAllHosts profiles (both 64-bit and 32-bit)
    instead of the current user's CurrentUserAllHosts profile.
    Requires the process to be elevated.

.EXAMPLE
    # Current-user registration (build-from-source)
    powershell -ExecutionPolicy Bypass -File register_pwsh_completion.ps1 -InstallDir "C:\ifm3d"

.EXAMPLE
    # All-users registration (NSIS installer, elevated)
    powershell -ExecutionPolicy Bypass -File register_pwsh_completion.ps1 -InstallDir "C:\Program Files\ifm3d" -AllUsers
#>
param(
    [Parameter(Mandatory = $true)]
    [string]$InstallDir,

    [switch]$AllUsers
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$resolvedInstallDir = [System.IO.Path]::GetFullPath($InstallDir)
$ifm3dExe = Join-Path $resolvedInstallDir 'bin\ifm3d.exe'
$installCompletionScript = Join-Path $resolvedInstallDir 'share\ifm3d\completions\ifm3d.ps1'

if ($AllUsers) {
    $principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltinRole]::Administrator)) {
        throw "AllUsers registration requires Administrator privileges."
    }
}

# Returns AllUsers profile paths for both 64-bit (System32) and 32-bit (SysWOW64)
# Windows PowerShell. Sysnative is used when running in a 32-bit process on a
# 64-bit OS so that the WOW64 file-system redirector is bypassed for System32.
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

# Select profile paths based on scope
if ($AllUsers) {
    $profilePaths = Get-AllUsersProfilePaths
} else {
    $profilePaths = @($PROFILE.CurrentUserAllHosts)
}

# Completion script location to source from profile:
# - All-users mode: use the installed script path in Program Files directly.
# - Current-user mode: generate a per-user copy to avoid write-permission issues.
if ($AllUsers) {
    $completionScriptToSource = $installCompletionScript
} else {
    $profileDir = Split-Path $profilePaths[0] -Parent
    $completionScriptToSource = Join-Path $profileDir 'ifm3d_completion.ps1'
}

$utf8NoBom = [System.Text.UTF8Encoding]::new($false)

$completionDir = Split-Path $completionScriptToSource -Parent
if (-not (Test-Path -LiteralPath $completionDir)) {
    New-Item -ItemType Directory -Path $completionDir -Force | Out-Null
}

# Keep registration in sync with the current install.
# Prefer generating from the installed binary; if unavailable, fall back to
# copying the installed completion script.
if (Test-Path -LiteralPath $ifm3dExe) {
    $completionOutput = & $ifm3dExe generate-completion pwsh
    $ifm3dExitCode = $LASTEXITCODE
    if ($ifm3dExitCode -ne 0) {
        throw "ifm3d generate-completion pwsh failed with exit code $ifm3dExitCode"
    }

    $completionContent = [string]::Join("`r`n", $completionOutput)
    [System.IO.File]::WriteAllText($completionScriptToSource, $completionContent, $utf8NoBom)
} elseif (Test-Path -LiteralPath $installCompletionScript) {
    if (-not ($completionScriptToSource -eq $installCompletionScript)) {
        Copy-Item -LiteralPath $installCompletionScript -Destination $completionScriptToSource -Force
    }
} else {
    throw "No completion source found. Checked: $ifm3dExe and $installCompletionScript"
}

# Sentinel markers make the block easy to find and update/remove.
$markerBegin = '# >>> ifm3d tab completion begin'
$markerEnd   = '# <<< ifm3d tab completion end'
$block = @"
$markerBegin
. '$completionScriptToSource'
$markerEnd
"@
$pattern = '(?ms)^\s*# >>> ifm3d tab completion begin\r?\n.*?\r?\n# <<< ifm3d tab completion end\s*'

foreach ($profilePath in $profilePaths) {
    $profileDir = Split-Path $profilePath -Parent
    if (-not (Test-Path -LiteralPath $profileDir)) {
        New-Item -ItemType Directory -Path $profileDir -Force | Out-Null
    }
    if (-not (Test-Path -LiteralPath $profilePath)) {
        New-Item -ItemType File -Path $profilePath -Force | Out-Null
    }

    $content = Get-Content -LiteralPath $profilePath -Raw -ErrorAction SilentlyContinue
    if ($null -eq $content) { $content = '' }

    # Idempotent and self-healing: remove stale blocks, append one current block.
    $stripped = [regex]::Replace($content, $pattern, '')
    $stripped = $stripped.TrimEnd("`r", "`n")

    if ([string]::IsNullOrEmpty($stripped)) {
        $newContent = $block + "`r`n"
    } else {
        $newContent = $stripped + "`r`n`r`n" + $block + "`r`n"
    }

    [System.IO.File]::WriteAllText($profilePath, $newContent, $utf8NoBom)
    if ($content -match [regex]::Escape($markerBegin)) {
        Write-Host "ifm3d: PowerShell tab completion updated in:"
    } else {
        Write-Host "ifm3d: PowerShell tab completion registered in:"
    }
    Write-Host "  $profilePath"
}
Write-Host "Restart PowerShell (or run '. `$PROFILE') to activate."
