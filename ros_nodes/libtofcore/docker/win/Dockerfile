# escape=`

# Use the latest Windows Server Core 2022 image.
#FROM mcr.microsoft.com/windows/servercore:ltsc2022
FROM mcr.microsoft.com/windows/servercore:ltsc2019
# Restore the default Windows shell for correct batch processing.
SHELL ["cmd", "/S", "/C"]

RUN `
    # Download the Build Tools bootstrapper.
    curl -SL --output vs_buildtools.exe https://aka.ms/vs/17/release/vs_buildtools.exe `
    `
    # Install Build Tools with the Microsoft.VisualStudio.Workload.AzureBuildTools workload, excluding workloads and components with known issues.
    && (start /w vs_buildtools.exe --quiet --wait --norestart --nocache `
        --installPath "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools" `
        --add    Microsoft.VisualStudio.Workload.ManagedDesktopBuildTools `
        --add    Microsoft.VisualStudio.Workload.VCTools --includeRecommended `
        --add    Microsoft.Component.MSBuild `
        --remove Microsoft.VisualStudio.Component.Windows10SDK.10240 `
        --remove Microsoft.VisualStudio.Component.Windows10SDK.10586 `
        --remove Microsoft.VisualStudio.Component.Windows10SDK.14393 `
        --remove Microsoft.VisualStudio.Component.Windows81SDK `
        || IF "%ERRORLEVEL%"=="3010" EXIT 0) `
    `
    # Cleanup
    && del /q vs_buildtools.exe

#Install Python and git
RUN powershell.exe -ExecutionPolicy RemoteSigned `
    irm get.scoop.sh -outfile 'scoop-install.ps1'; `
    .\scoop-install.ps1 -RunAsAdmin; `
    scoop install git; `
    rm scoop-install.ps1

#Install Python 3.11.5
RUN powershell.exe -Command `
  $ErrorActionPreference = 'Stop'; `
  $ProgressPreference = 'SilentlyContinue'; `
  Invoke-WebRequest https://www.python.org/ftp/python/3.11.5/python-3.11.5-amd64.exe -OutFile c:\python-3.11.5-amd64.exe ; `
  Start-Process c:\python-3.11.5-amd64.exe -ArgumentList '/quiet InstallAllUsers=1 PrependPath=1' -Wait ; `
  Remove-Item c:\python-3.11.5-amd64.exe -Force

#Install Python 3.8.10
RUN powershell.exe -Command `
  $ErrorActionPreference = 'Stop'; `
  $ProgressPreference = 'SilentlyContinue'; `
  Invoke-WebRequest https://www.python.org/ftp/python/3.8.10/python-3.8.10-amd64.exe -OutFile c:\python-3.8.10-amd64.exe ; `
  Start-Process c:\python-3.8.10-amd64.exe -ArgumentList '/quiet InstallAllUsers=1 PrependPath=1' -Wait ; `
  Remove-Item c:\python-3.8.10-amd64.exe -Force



#Install vcpkg and prime it with some boost libraries
WORKDIR C:\
RUN powershell.exe -ExecutionPolicy RemoteSigned `
    git clone https://github.com/Microsoft/vcpkg.git; `
    git config --global --add safe.directory *; `
    c:\vcpkg\bootstrap-vcpkg.bat; `
    c:\vcpkg\vcpkg --triplet x64-windows-static install `
        boost-system boost-program-options boost-asio boost-endian boost-scope-exit boost-signals2 pybind11

WORKDIR c:\build
# Define the entry point for the docker container.
# This entry point starts the developer command prompt and launches the PowerShell shell.
ENTRYPOINT ["C:\\Program Files (x86)\\Microsoft Visual Studio\\2022\\BuildTools\\Common7\\Tools\\VsDevCmd.bat", "-arch=amd64", "&&", "powershell.exe", "-NoLogo", "-ExecutionPolicy", "Bypass"]