param([switch]$NoPackage, [string]$GameExecutable)
$ErrorActionPreference = 'Stop'
$buildRoot = $PSScriptRoot
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Install Visual Studio 2022 or Build Tools with Desktop development with C++ (v143).' }
$msbuild = @(& $vswhere -latest -products '*' -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe') | Select-Object -First 1
if (-not $msbuild) { throw 'MSBuild was not found.' }
$python = (Get-Command python -ErrorAction Stop).Source
Push-Location $buildRoot
try {
    & $python tools/rpt_tool.py prepare
    if ($LASTEXITCODE -ne 0) { throw 'Asset preparation failed.' }
    foreach ($project in @('ETS2ReverseEntityRuntime', 'GuidanceTests', 'BuildProfileTests')) {
        & $msbuild "src/entity_runtime/$project.vcxproj" /nologo /m /p:Configuration=Release /p:Platform=x64 /verbosity:minimal
        if ($LASTEXITCODE -ne 0) { throw "$project build failed." }
    }
    foreach ($test in @('build/guidance_tests/GuidanceTests.exe', 'build/entity_runtime_tests/BuildProfileTests.exe')) {
        & "./$test"
        if ($LASTEXITCODE -ne 0) { throw "$test failed." }
    }
    if ($GameExecutable) {
        $gamePath = (Resolve-Path -LiteralPath $GameExecutable).Path
        $gameHash = (Get-FileHash -LiteralPath $gamePath -Algorithm SHA256).Hash
        & './build/entity_runtime_tests/BuildProfileTests.exe' $gamePath $gameHash
        if ($LASTEXITCODE -ne 0) { throw 'Installed game executable compatibility checks failed.' }
    }
    & $python tools/rpt_tool.py package
    if ($LASTEXITCODE -ne 0) { throw 'Plugin packaging failed.' }
    & $python tools/rpt_tool.py validate
    if ($LASTEXITCODE -ne 0) { throw 'Resource validation failed.' }
    if (-not $NoPackage) {
        & $python tools/build_release.py
        if ($LASTEXITCODE -ne 0) { throw 'Release archive creation failed.' }
    }
} finally {
    Pop-Location
}
