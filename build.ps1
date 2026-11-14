param(
    [Parameter(Position = 0)]
    [ValidateSet('sd', 'nosd')]
    [string]$Variant = 'sd',

    [string]$NcsRoot = 'C:\ncs\v3.2.4',

	[switch]$Diagnostic,

	[switch]$DiagnosticBle,

    [switch]$EncoderStartupDiagnostic
)

$ErrorActionPreference = 'Stop'

$workspace = $PSScriptRoot
$applicationName = Split-Path $workspace -Leaf
$zephyrBase = Join-Path $NcsRoot 'zephyr'
$ncsVersion = Split-Path $NcsRoot -Leaf
$ncsHome = Split-Path $NcsRoot -Parent
$toolchainCatalogPath = Join-Path $ncsHome 'toolchains\toolchains.json'

if (-not (Test-Path -LiteralPath $zephyrBase)) {
    throw "Zephyr was not found at '$zephyrBase'. Pass the installed NCS path with -NcsRoot."
}

if (-not (Test-Path -LiteralPath $toolchainCatalogPath)) {
    throw "The NCS toolchain catalog was not found at '$toolchainCatalogPath'."
}

$catalog = @(Get-Content -LiteralPath $toolchainCatalogPath -Raw | ConvertFrom-Json)
$toolchainEntry = $catalog[0].toolchains |
    Where-Object { $_.ncs_versions -contains $ncsVersion } |
    Select-Object -First 1

if (-not $toolchainEntry) {
    throw "No installed toolchain is registered for NCS $ncsVersion."
}

$bundleId = $toolchainEntry.identifier.bundle_id
$toolchain = Join-Path (Join-Path $ncsHome 'toolchains') $bundleId
$west = Join-Path $toolchain 'opt\bin\Scripts\west.exe'

if (-not (Test-Path -LiteralPath $west)) {
    throw "west.exe was not found at '$west'."
}

$toolPaths = @(
    $toolchain,
    (Join-Path $toolchain 'mingw64\bin'),
    (Join-Path $toolchain 'opt\bin'),
    (Join-Path $toolchain 'opt\bin\Scripts'),
    (Join-Path $toolchain 'opt\nanopb\generator-bin'),
    (Join-Path $toolchain 'nrfutil\bin'),
    (Join-Path $toolchain 'opt\zephyr-sdk\arm-zephyr-eabi\bin'),
    (Join-Path $toolchain 'opt\zephyr-sdk\riscv64-zephyr-elf\bin')
)

$env:PATH = ($toolPaths -join ';') + ';' + $env:PATH
$env:PYTHONPATH = @(
    (Join-Path $toolchain 'opt\bin'),
    (Join-Path $toolchain 'opt\bin\Lib'),
    (Join-Path $toolchain 'opt\bin\Lib\site-packages')
) -join ';'
$env:NRFUTIL_HOME = Join-Path $toolchain 'nrfutil\home'
$env:ZEPHYR_BASE = $zephyrBase
$env:ZEPHYR_TOOLCHAIN_VARIANT = 'zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR = Join-Path $toolchain 'opt\zephyr-sdk'

$board = 'promicro_nrf52840/nrf52840/uf2'
$diagnosticBuild = $Diagnostic -or $DiagnosticBle -or $EncoderStartupDiagnostic

if ($Variant -eq 'nosd') {
	$variantSuffix = '-nosd'
	$uf2VariantSuffix = '_nosd'
	$configurationFiles = @(
		((Join-Path $workspace 'prj.conf') -replace '\\', '/'),
		((Join-Path $workspace 'prj_nosd.conf') -replace '\\', '/')
	)
	} else {
	$variantSuffix = ''
	$uf2VariantSuffix = ''
	$configurationFiles = @(
		((Join-Path $workspace 'prj.conf') -replace '\\', '/')
	)
}

if ($EncoderStartupDiagnostic) {
	$buildDirectory = Join-Path $workspace ("build{0}-startup-diagnostic" -f $variantSuffix)
	$firmwareName = "blescroll{0}_startup_diagnostic.uf2" -f $uf2VariantSuffix
} elseif ($DiagnosticBle) {
	$buildDirectory = Join-Path $workspace ("build{0}-diagnostic-ble" -f $variantSuffix)
	$firmwareName = "blescroll{0}_diagnostic_ble.uf2" -f $uf2VariantSuffix
} elseif ($diagnosticBuild) {
	$buildDirectory = Join-Path $workspace ("build{0}-diagnostic" -f $variantSuffix)
	$firmwareName = "blescroll{0}_diagnostic.uf2" -f $uf2VariantSuffix
} else {
	$buildDirectory = Join-Path $workspace ("build{0}" -f $variantSuffix)
	$firmwareName = "blescroll{0}.uf2" -f $uf2VariantSuffix
}

$firmware = Join-Path (Join-Path $buildDirectory $applicationName) $firmwareName

if ($diagnosticBuild) {
    if ($configurationFiles.Count -eq 0) {
        $configurationFiles += ((Join-Path $workspace 'prj.conf') -replace '\\', '/')
    }
	$configurationFiles += ((Join-Path $workspace 'prj_diagnostic.conf') -replace '\\', '/')
}

if ($DiagnosticBle) {
	$configurationFiles += ((Join-Path $workspace 'prj_diagnostic_ble.conf') -replace '\\', '/')
}

if ($EncoderStartupDiagnostic) {
    $configurationFiles += ((Join-Path $workspace 'prj_encoder_startup_diagnostic.conf') -replace '\\', '/')
}

$configuration = if ($configurationFiles.Count -gt 0) {
    '-DCONF_FILE={0}' -f ($configurationFiles -join ';')
} else {
    $null
}

$westArguments = @(
    '-z', $zephyrBase,
    'build',
    '--sysbuild',
    '-p', 'auto',
    '-b', $board,
    '-d', $buildDirectory,
    $workspace
)

if ($configuration) {
    $westArguments += @('--', $configuration)
}

Write-Host "Building '$Variant' firmware with NCS $ncsVersion..."
& $west @westArguments
if ($LASTEXITCODE -ne 0) {
    throw "west build failed with exit code $LASTEXITCODE."
}

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "The expected UF2 file was not generated at '$firmware'."
}

$validator = Join-Path $workspace 'tools\verify_uf2.py'
& python $validator $firmware --variant $Variant
if ($LASTEXITCODE -ne 0) {
    throw "UF2 validation failed with exit code $LASTEXITCODE."
}

Write-Host ''
Write-Host "Firmware:     $firmware" -ForegroundColor Green
