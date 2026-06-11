# verify.ps1 -- thin driver for zw_verify: finds the freshest built exe,
# puts the project-staged OCCT DLLs on PATH (the loose OCCT install under
# D:\tools may not match the import libs the exe linked), runs the verify,
# and on FAIL points at / runs the bisect when the snapshot carries _state.
#
#   .\verify.ps1 <part.cax.json>            # full verify
#   .\verify.ps1 <part.cax.json> -Bisect    # bisect immediately
#   .\verify.ps1 <part.cax.json> -Args '--states','all'
param(
    [Parameter(Mandatory = $true)] [string]$Json,
    [switch]$Bisect,
    [string[]]$Args = @()
)
$ErrorActionPreference = 'Stop'

$outRoot = 'D:\projects\tantien-cad\out'
$exe = Get-ChildItem "$outRoot\thirdparty\cax\tools\zw_verify" -Recurse -Filter zw_verify.exe |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $exe) { throw "zw_verify.exe not built; cmake --build $outRoot --target zw_verify" }

# Same-config runtime dir carries the matching OCCT DLLs.
$cfg = Split-Path $exe.DirectoryName -Leaf
$env:PATH = "$outRoot\$cfg;$env:PATH"

$cmd = @($Json)
if ($Bisect) { $cmd += '--bisect' }
$cmd += $Args

& $exe.FullName @cmd
$code = $LASTEXITCODE

if ($code -eq 1 -and -not $Bisect) {
    # FAIL: bisect right away when the snapshot has per-feature truth.
    $hasState = Select-String -Path $Json -Pattern '"_state"' -Quiet
    if ($hasState) {
        Write-Host "`n--- FAIL with _state truth present: bisecting ---`n"
        & $exe.FullName $Json --bisect
    } else {
        Write-Host "`nSnapshot has no _state blocks. Re-export with the new CaxExport.dll"
        Write-Host "(optionally CAX_FEAT_STATE=2 for volume truth, CAX_FEAT_STATE_STEP=K1,K2"
        Write-Host "for cumulative state STEPs), then run: .\verify.ps1 <json> -Bisect"
    }
}
exit $code
