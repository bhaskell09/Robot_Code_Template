# Syncs .github/skills -> .claude/skills
#
# The same skills have to exist in two places: GitHub Copilot reads .github/skills,
# Claude Code reads .claude/skills. Neither tool will look in the other's directory,
# and Windows symlinks are too fragile to rely on across everyone's laptops.
#
# So .github/skills is the canonical copy. Edit there, then run this.
#
#   pwsh scripts/sync-skills.ps1          # copy .github/skills -> .claude/skills
#   pwsh scripts/sync-skills.ps1 -Check   # report drift without changing anything
#
# -Check exits non-zero when the two are out of sync, so it also works as a CI step.

param([switch]$Check)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$source   = Join-Path $repoRoot '.github/skills'
$dest     = Join-Path $repoRoot '.claude/skills'

if (-not (Test-Path $source)) {
    Write-Error "Source not found: $source"
}

$skills = Get-ChildItem -Path $source -Directory
if ($skills.Count -eq 0) {
    Write-Error "No skills found in $source"
}

$drift = @()

foreach ($skill in $skills) {
    $from = Join-Path $skill.FullName 'SKILL.md'
    $to   = Join-Path $dest "$($skill.Name)/SKILL.md"

    if (-not (Test-Path $from)) {
        Write-Warning "$($skill.Name): no SKILL.md, skipping"
        continue
    }

    $same = (Test-Path $to) -and
            ((Get-FileHash $from).Hash -eq (Get-FileHash $to).Hash)

    if ($same) {
        Write-Host "  ok      $($skill.Name)"
        continue
    }

    $drift += $skill.Name

    if ($Check) {
        Write-Host "  DRIFT   $($skill.Name)" -ForegroundColor Yellow
    } else {
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $to) | Out-Null
        Copy-Item $from $to -Force
        Write-Host "  synced  $($skill.Name)" -ForegroundColor Green
    }
}

# A skill deleted from .github but still present in .claude would keep being offered
# to Claude Code, pointing at files that may no longer exist.
if (Test-Path $dest) {
    $names = $skills.Name
    foreach ($stale in Get-ChildItem -Path $dest -Directory | Where-Object { $names -notcontains $_.Name }) {
        $drift += $stale.Name
        if ($Check) {
            Write-Host "  STALE   $($stale.Name) (not in .github/skills)" -ForegroundColor Yellow
        } else {
            Remove-Item $stale.FullName -Recurse -Force
            Write-Host "  removed $($stale.Name) (not in .github/skills)" -ForegroundColor Green
        }
    }
}

if ($Check -and $drift.Count -gt 0) {
    Write-Host ""
    Write-Error "$($drift.Count) skill(s) out of sync. Run: pwsh scripts/sync-skills.ps1"
}

Write-Host ""
Write-Host "Skills in sync."
