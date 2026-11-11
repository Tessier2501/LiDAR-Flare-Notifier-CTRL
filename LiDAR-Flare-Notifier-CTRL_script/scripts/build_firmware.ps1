param(
    [ValidateSet('Debug', 'Release')]
    [string[]]$Preset = @('Debug', 'Release'),
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'

$scriptRepo = Split-Path -Parent $PSScriptRoot
$workspace = Split-Path -Parent $scriptRepo
$firmwareRoot = Join-Path $workspace 'LiDAR-Flare-Notifier-CTRL'
$buildRoot = Join-Path $scriptRepo 'temp\build'
$bundleRoot = if ($env:STM32CUBE_BUNDLES) {
    $env:STM32CUBE_BUNDLES
} else {
    Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
}

function Resolve-BundleBin {
    param([string]$Name)

    $bundleDir = Join-Path $bundleRoot $Name
    $versionDir = Get-ChildItem -LiteralPath $bundleDir -Directory |
        Sort-Object { [version](($_.Name -split '\+')[0]) } -Descending |
        Select-Object -First 1

    if ($null -eq $versionDir) {
        throw "bundle has no version directory: $bundleDir"
    }

    $binDir = Join-Path $versionDir.FullName 'bin'
    if (-not (Test-Path -LiteralPath $binDir)) {
        throw "bundle bin directory not found: $binDir"
    }

    return $binDir
}

$bundleBins = @(
    Resolve-BundleBin 'gnu-tools-for-stm32'
    Resolve-BundleBin 'cmake'
    Resolve-BundleBin 'ninja'
)

$env:PATH = ($bundleBins -join [IO.Path]::PathSeparator) + [IO.Path]::PathSeparator + $env:PATH

foreach ($command in @('cmake', 'ninja', 'arm-none-eabi-gcc', 'arm-none-eabi-objcopy', 'arm-none-eabi-size')) {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) {
        throw "required command not found: $command"
    }
}

Push-Location $firmwareRoot
try {
    foreach ($presetName in $Preset) {
        if ($Clean) {
            $presetBuildDir = Join-Path $buildRoot $presetName
            if (Test-Path -LiteralPath $presetBuildDir) {
                Remove-Item -LiteralPath $presetBuildDir -Recurse -Force
            }
        }

        Write-Output "preset=$presetName"
        & cmake --preset $presetName
        if ($LASTEXITCODE -ne 0) {
            throw "configure failed: $presetName"
        }

        & cmake --build --preset $presetName
        if ($LASTEXITCODE -ne 0) {
            throw "build failed: $presetName"
        }

        Write-Output "build_dir=$(Join-Path $buildRoot $presetName)"
    }
} finally {
    Pop-Location
}