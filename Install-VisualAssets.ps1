<#
.SYNOPSIS
    Install or restore the standalone ETS2 Reverse Guidance enhanced visual assets pack.
.DESCRIPTION
    Copies enhanced 64x64 semantic DDS textures and unified day/night MAT files from
    assets_improved/home/ets2_reverse_posture_tmp/ into the player's ETS2 user home
    directory, automatically backing up overwritten files first. Does not modify the
    game installation, saves, DLL, or INI. The plugin DLL remains 100% functional
    without this visual pack.
.PARAMETER GameHome
    Path to the ETS2 user profile directory (default: Documents\Euro Truck Simulator 2,
    checking OneDrive fallback automatically).
.PARAMETER Restore
    Restore previously backed-up files from the latest backup (or -BackupDir).
.PARAMETER BackupDir
    Optional explicit backup directory to restore from when -Restore is specified.
.PARAMETER DryRun
    Preview planned operations without writing or modifying any files.
#>
param(
    [string]$GameHome,
    [switch]$Restore,
    [string]$BackupDir,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
$repoRoot = $PSScriptRoot
$packRoot = Join-Path $repoRoot 'assets_improved\home\ets2_reverse_posture_tmp'

function Resolve-Ets2Home([string]$explicit) {
    if ($explicit) {
        if (-not (Test-Path -LiteralPath $explicit)) {
            throw "Specified -GameHome does not exist: $explicit"
        }
        return (Resolve-Path -LiteralPath $explicit).Path
    }
    $candidates = @(
        (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Euro Truck Simulator 2'),
        (Join-Path $env:USERPROFILE 'OneDrive\Documents\Euro Truck Simulator 2')
    ) | Where-Object { $_ -and (Test-Path -LiteralPath $_) }
    $selected = $candidates | Select-Object -First 1
    if (-not $selected) {
        throw 'Could not locate Euro Truck Simulator 2 user directory. Pass -GameHome explicitly.'
    }
    return $selected
}

$ets2Home = Resolve-Ets2Home $GameHome
$targetRoot = Join-Path $ets2Home 'ets2_reverse_posture_tmp'
$backupBase = Join-Path $ets2Home 'ets2_reverse_posture_tmp_backups'

if ($Restore) {
    if (-not $BackupDir) {
        if (-not (Test-Path -LiteralPath $backupBase)) {
            throw "No visual asset backups found under: $backupBase"
        }
        $latest = Get-ChildItem -LiteralPath $backupBase -Directory |
            Sort-Object Name -Descending | Select-Object -First 1
        if (-not $latest) {
            throw "No backup snapshots found under: $backupBase"
        }
        $BackupDir = $latest.FullName
    }
    if (-not (Test-Path -LiteralPath $BackupDir)) {
        throw "Backup directory does not exist: $BackupDir"
    }
    $backupFiles = Get-ChildItem -LiteralPath $BackupDir -File -Recurse
    if (-not $backupFiles) {
        throw "Backup directory contains no files: $BackupDir"
    }
    foreach ($file in $backupFiles) {
        $rel = $file.FullName.Substring($BackupDir.Length).TrimStart('\', '/')
        $dst = Join-Path $targetRoot $rel
        if ($DryRun) {
            Write-Output "[DryRun] Restore: $rel -> $dst"
        } else {
            $dstDir = Split-Path -Parent $dst
            if (-not (Test-Path -LiteralPath $dstDir)) {
                New-Item -ItemType Directory -Path $dstDir -Force | Out-Null
            }
            Copy-Item -LiteralPath $file.FullName -Destination $dst -Force
        }
    }
    if (-not $DryRun) {
        Write-Output "Restored $($backupFiles.Count) file(s) from $BackupDir into $targetRoot."
    }
    exit 0
}

if (-not (Test-Path -LiteralPath $packRoot)) {
    throw "Visual assets pack not found: $packRoot"
}
if (-not (Test-Path -LiteralPath $targetRoot)) {
    throw "Target plugin asset folder does not exist yet: $targetRoot. Install the base plugin assets first."
}

$packFiles = Get-ChildItem -LiteralPath $packRoot -File -Recurse
if (-not $packFiles) {
    throw "No files found inside: $packRoot"
}

$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$snapshotDir = Join-Path $backupBase $timestamp
$backedUp = 0

foreach ($file in $packFiles) {
    $rel = $file.FullName.Substring($packRoot.Length).TrimStart('\', '/')
    $dst = Join-Path $targetRoot $rel
    if (Test-Path -LiteralPath $dst) {
        $bak = Join-Path $snapshotDir $rel
        if ($DryRun) {
            Write-Output "[DryRun] Backup:  $dst -> $bak"
        } else {
            $bakDir = Split-Path -Parent $bak
            if (-not (Test-Path -LiteralPath $bakDir)) {
                New-Item -ItemType Directory -Path $bakDir -Force | Out-Null
            }
            Copy-Item -LiteralPath $dst -Destination $bak -Force
        }
        $backedUp++
    }
    if ($DryRun) {
        Write-Output "[DryRun] Install: $rel -> $dst"
    } else {
        $dstDir = Split-Path -Parent $dst
        if (-not (Test-Path -LiteralPath $dstDir)) {
            New-Item -ItemType Directory -Path $dstDir -Force | Out-Null
        }
        Copy-Item -LiteralPath $file.FullName -Destination $dst -Force
    }
}

if ($DryRun) {
    Write-Output "[DryRun] Planned install of $($packFiles.Count) file(s) ($backedUp backup(s)) into $targetRoot."
} else {
    Write-Output "Installed $($packFiles.Count) enhanced visual asset file(s) into $targetRoot."
    if ($backedUp -gt 0) {
        Write-Output "Backed up $backedUp original file(s) to $snapshotDir."
        Write-Output "To roll back, run: .\Install-VisualAssets.ps1 -Restore"
    }
}
