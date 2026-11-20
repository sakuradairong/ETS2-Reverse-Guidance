$ErrorActionPreference = 'Stop'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$msbuild = @(& $vswhere -latest -products '*' -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe') | Select-Object -First 1
if (-not $msbuild) { throw 'MSBuild with C++ v143 build tools is required.' }
$project = Join-Path $PSScriptRoot 'src\entity_runtime\ETS2ReverseEntityRuntime.vcxproj'
& $msbuild $project /nologo /m /p:Configuration=Release /p:Platform=x64 /verbosity:minimal
if ($LASTEXITCODE -ne 0) { throw 'DLL compilation failed.' }
Write-Output 'DLL compiled. No test programs, game or plugin were executed.'
