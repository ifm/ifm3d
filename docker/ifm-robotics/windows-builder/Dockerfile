# escape=`

# Use the latest Windows Server 2019 image.
ARG BASE_IMAGE=mcr.microsoft.com/windows:ltsc2019

# List of python versions to install, separated by ';'
ARG PYTHON_VERSIONS=3.8.10;3.9.13;3.10.11;3.11.7;3.12.1

FROM ${BASE_IMAGE}
ARG PYTHON_VERSIONS

SHELL ["cmd", "/S", "/C"]

RUN `
    # Download the Build Tools bootstrapper.
    curl -SL --output vs_buildtools.exe https://aka.ms/vs/16/release/vs_buildtools.exe `
    `
    # Install Build Tools with the Microsoft.VisualStudio.Workload.VCTools workload.
    && (start /w vs_buildtools.exe --quiet --wait --norestart --nocache `
        --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended `
        --installPath C:\BuildTools `
        || IF "%ERRORLEVEL%"=="3010" EXIT 0) `
    `
    # Cleanup
    && del /q vs_buildtools.exe

# Download pwsh
RUN `
    curl -SL --output pwsh.zip https://github.com/PowerShell/PowerShell/releases/download/v7.3.3/PowerShell-7.3.3-win-x64.zip `
    && mkdir C:\pwsh `
    && tar -xf pwsh.zip -C C:\pwsh\ `
    && del /q pwsh.zip

SHELL ["C:\\pwsh\\pwsh.exe", "-NoLogo", "-ExecutionPolicy", "Bypass", "-Command"]

# Download cmake
RUN `
    curl -SL --output cmake.zip https://github.com/Kitware/CMake/releases/download/v3.26.3/cmake-3.26.3-windows-x86_64.zip `
    && mkdir C:\cmake `
    && tar -xf cmake.zip --strip-components 1 -C C:\cmake\ `
    && del cmake.zip

# Download git
RUN `
    curl -SL --output git-setup.exe https://github.com/git-for-windows/git/releases/download/v2.40.0.windows.1/Git-2.40.0-64-bit.exe `
    && cmd /C start /w git-setup.exe /VERYSILENT /NORESTART /NOCANCEL /SP- /CLOSEAPPLICATIONS /RESTARTAPPLICATIONS /COMPONENTS='icons,ext\reg\shellhere,assoc,assoc_sh' `
    && del git-setup.exe

# Download and set up all python versions
RUN `
    curl -SL --output nuget.exe "https://dist.nuget.org/win-x86-commandline/latest/nuget.exe"; `
    foreach ($PYTHON_VERSION in $env:PYTHON_VERSIONS.Split(';')) { `
        ./nuget.exe install python -Version $PYTHON_VERSION -OutputDirectory C:\python `
        && & C:\python\python.${PYTHON_VERSION}\tools\python.exe -m pip install wheel `
    }; `
    del nuget.exe

# Download NSIS
RUN `
    curl -SL --output nsis.zip https://downloads.sourceforge.net/project/nsis/NSIS%203/3.08/nsis-3.08.zip `
    && mkdir C:\nsis `
    && tar -xf nsis.zip --strip-components 1 -C C:\nsis\ `
    && del nsis.zip

# Download googletest
Run & 'C:\Program Files\Git\bin\git.exe' clone https://github.com/google/googletest.git C:\googletest

# Add everything to the PATH
USER ContainerAdministrator
RUN setx /M PATH \"$env:PATH;C:\Program Files\Git\bin;C:\cmake\bin;C:\pwsh;C:\nsis\"

# Define the entry point for the docker container.
# This entry point starts the developer command prompt and launches the PowerShell shell.
ENTRYPOINT ["C:\\BuildTools\\Common7\\Tools\\VsDevCmd.bat", "&&", "pwsh.exe", "-NoLogo", "-ExecutionPolicy", "Bypass"]
