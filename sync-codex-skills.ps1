param(
    [string]$SourceDir = (Join-Path $PSScriptRoot "skills"),
    [string]$CodexHome = $(if ($env:CODEX_HOME) { $env:CODEX_HOME } else { Join-Path $HOME ".codex" })
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Remove-Utf8Bom {
    param([Parameter(Mandatory = $true)][string]$Path)

    $bytes = [System.IO.File]::ReadAllBytes($Path)
    if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
        if ($bytes.Length -eq 3) {
            [System.IO.File]::WriteAllBytes($Path, [byte[]]@())
        } else {
            [System.IO.File]::WriteAllBytes($Path, $bytes[3..($bytes.Length - 1)])
        }
        return $true
    }

    return $false
}

function Get-TextFiles {
    param([Parameter(Mandatory = $true)][string]$Root)

    return Get-ChildItem -LiteralPath $Root -Recurse -File | Where-Object {
        $_.Extension -in @('.md', '.yaml', '.yml')
    }
}

if (-not (Test-Path -LiteralPath $SourceDir)) {
    throw "Source skills directory not found: $SourceDir"
}

$destRoot = Join-Path $CodexHome "skills"
New-Item -ItemType Directory -Force -Path $destRoot | Out-Null

$skillDirs = Get-ChildItem -LiteralPath $SourceDir -Directory | Where-Object {
    Test-Path -LiteralPath (Join-Path $_.FullName "SKILL.md")
}

if (-not $skillDirs -or $skillDirs.Count -eq 0) {
    Write-Host "No skill folders with SKILL.md found under: $SourceDir"
    exit 0
}

$skillsSynced = 0
$sourceBomRemoved = 0
$destBomRemoved = 0

foreach ($skill in $skillDirs) {
    $sourceTextFiles = Get-TextFiles -Root $skill.FullName
    foreach ($file in $sourceTextFiles) {
        if (Remove-Utf8Bom -Path $file.FullName) {
            $sourceBomRemoved++
        }
    }

    $destSkillDir = Join-Path $destRoot $skill.Name
    New-Item -ItemType Directory -Force -Path $destSkillDir | Out-Null

    Get-ChildItem -LiteralPath $skill.FullName -Force | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $destSkillDir -Recurse -Force
    }

    $destTextFiles = Get-TextFiles -Root $destSkillDir
    foreach ($file in $destTextFiles) {
        if (Remove-Utf8Bom -Path $file.FullName) {
            $destBomRemoved++
        }
    }

    $skillsSynced++
}

Write-Host "Synced $skillsSynced skill(s) from '$SourceDir' to '$destRoot'."
Write-Host "Removed BOM in source files: $sourceBomRemoved"
Write-Host "Removed BOM in destination files: $destBomRemoved"
