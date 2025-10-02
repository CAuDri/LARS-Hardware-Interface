# CAuDri - PowerShell script to setup the bus-system-ros2 Dev Container on Windows.
# 
# Needs to be run as Administrator.
#
# The script will check if the following tools are installed:
# - Windows Subsystem for Linux (WSL)
#   - Ubuntu-24.04 WSL distro
# - Docker
# - VS Code
# - usbipd
# - OpenSSH Client
# 
# For some tools the script can install them automatically after asking the user for permission.
#
# The script will check for the existence of an SSH key and start the SSH Agent service.
# This is necessary to pass the SSH key to the Dev Container for GitLab access.
# If for security reasons you do not want to pass your SSH keys to the Dev Container, you can skip this step.
#  
# It will install the Ubuntu-24.04 WSL distro if the Docker Desktop distro is set as default.
# This is necessary to add a udev rule to the WSL, giving users in the Dev Container access to serial devices.
# If you have another WSL distro set as default, you can skip this step.
# The script will then run 'add_udev_rules_host.sh' to add the udev rules to the WSL distro.

$sshKeyPath = "$ENV:USERPROFILE\.ssh\id_rsa"

$wslDistro = "Ubuntu-24.04"
$udevBashScriptPath = ".devcontainer/scripts/add_udev_rules_host.sh"

$sshTestServer = "git@git.kitcar-team.de"

# Check if the script is running as Administrator
if (-NOT ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host -ForegroundColor DarkRed "This script must be run as an Administrator. Please rerun the script with Administrator rights."
    Write-Host -ForegroundColor DarkRed "Make sure to enable the execution of PowerShell Scripts in your current session by running:"
    Write-Host -ForegroundColor DarkGreen "Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process"
    exit
}

# Clear the console
Clear-Host

Write-Host -ForegroundColor DarkGreen "`n-----Running CAuDri Dev Container Setup Script for Windows-----"
Write-Host -ForegroundColor DarkGreen "`nChecking for required tools..."

$setupSuccessful = $true
$softwareInstalled = $false

# Check for Docker installation
try {
    $dockerVersion = docker --version 2>&1
    if ($dockerVersion -match 'Docker version (\d+\.\d+\.\d+), build') {
        $installedVersion = [Version]$matches[1]
        Write-Host "Docker version: $installedVersion"
    }
    else {
        Write-Host -ForegroundColor DarkRed "Failed to determine the installed Docker version."
        $setupSuccessful = $false
    }
}
catch {
    Write-Host -ForegroundColor DarkRed "`nNo Docker installation found."
    $installDocker = Read-Host "`nInstall Docker Desktop? This is required to run the Dev Container. (yes/no)`n"
    if ($installDocker -eq "yes" -or $installDocker -eq "y" -or $installDocker -eq "Y") {
        # Install Docker Desktop
        Write-Host "Make sure to follow the setup instructions in README.md"
        try {
            Start-Process winget -ArgumentList "install --id Docker.DockerDesktop -e" -Wait
            Write-Host -ForegroundColor Green "Docker installed successfully."
            $softwareInstalled = $true
            Write-Host -ForegroundColor Red    "`n----------------------------------------------------------------------------------------------------------------"
            Write-Host -ForegroundColor Yellow "Docker Desktop was sucessfully installed. You might need to restart your computer to complete the installation."
            Write-Host -ForegroundColor Yellow "Afterwards, please rerun the script.`n" 
            Write-Host -ForegroundColor Yellow "Start Docker-Desktop to complete the setup. Continue after the docker engine is running."
            Write-Host -ForegroundColor Red    "----------------------------------------------------------------------------------------------------------------"
            Read-Host "Read the big yellow text? Press Enter to continue..."}
        catch {
            Write-Host -ForegroundColor DarkRed "Failed to install Docker, try installing manually."
            $setupSuccessful = $false
        }
    }
    else {
        Write-Host -ForegroundColor DarkRed "Install Docker to run the Dev Container."
        $setupSuccessful = $false
    }
}

# Check for VS Code installation
try {
    Write-Host -NoNewline "VS Code version: "
    code --version | Select-Object -First 1
}
catch {
    Write-Host -ForegroundColor DarkRed "`nNo VS Code installation found."
    $installVSCode = Read-Host "`nInstall VS Code? (yes/no)`n"
    if ($installVSCode -eq "yes" -or $installVSCode -eq "y" -or $installVSCode -eq "Y") {
        # Install VS Code
        try {
            Start-Process winget -ArgumentList "install vscode" -Wait
            Write-Host -ForegroundColor Green "VS Code installed successfully."
            $softwareInstalled = $true
        }
        catch {
            Write-Host -ForegroundColor DarkRed "Failed to install VS Code, try installing manually."
            $setupSuccessful = $false
        }
    }
    else {
        Write-Host -ForegroundColor DarkRed "Installing VS Code is necessary for working with the Dev Container."
        $setupSuccessful = $false
    }
}

# Check for usbipd installation
try {
    $usbipdVersion = usbipd --version 2>&1
    if ($usbipdVersion -match '(\d+\.\d+\.\d+)') {
        $installedVersion = [Version]$matches[1]
        Write-Host "usbipd version: $installedVersion"
    }
    else {
        Write-Host -ForegroundColor DarkRed "Failed to determine the installed usbipd version."
        $setupSuccessful = $false
    }
}
catch {
    Write-Host -ForegroundColor DarkRed "`nusbipd does not appear to be installed"
    $installUsbipd = Read-Host "`nInstall usbipd? This will be necessary to pass serial devices to the Dev Container. (yes/no)`n"
    if ($installUsbipd -eq "yes" -or $installUsbipd -eq "y" -or $installUsbipd -eq "Y") {
        # Install usbipd
        try {
            Start-Process winget -ArgumentList "install usbipd" -Wait
            Write-Host -ForegroundColor Green "usbipd installed successfully."
            $softwareInstalled = $true
        }
        catch {
            Write-Host -ForegroundColor DarkRed "Failed to install usbipd, try installing manually."
        }
    }
    else {
        Write-Host -ForegroundColor DarkRed "Install usbipd if you want to pass serial devices to the Dev Container."
        Write-Host -ForegroundColor DarkRed "This allows for flashing and debugging the firmware from within the Container."
        Write-Host -ForegroundColor DarkRed "More information can be found in the README.md."
    }
}

# Check for OpenSSH Client version 
# Uncomment if you are using a private repository and want to share your host SSH keys with the container

# try {
#     $openSSHVersion = ssh -V 2>&1
#     if ($openSSHVersion -match "OpenSSH_for_Windows_(\d+\.\d+)") {
#         $version = $matches[1]
#         $minimumVersion = [Version]"8.9"
#         $installedVersion = [Version]$version
#         Write-Host "OpenSSH version: $installedVersion"
#         if ($installedVersion -le $minimumVersion) {
#             Write-Host -ForegroundColor Red    "--------------------------------------------------------------------------------------------------------------"
#             Write-Host -ForegroundColor Yellow "You are using an old version of OpenSSH. This may cause issues when passing the SSH key to the Dev Container." 
#             Write-Host -ForegroundColor Yellow "Please update OpenSSH to version $minimumVersion or later."
#             Write-Host -ForegroundColor Yellow "The newest release can be found at https://github.com/PowerShell/Win32-OpenSSH"
#             Write-Host -ForegroundColor Red    "--------------------------------------------------------------------------------------------------------------"
#         }
#     }
#     else {
#         Write-Host -ForegroundColor DarkRed "Failed to determine the installed OpenSSH version."
#         $setupSuccessful = $false
#     }
# }
# catch {
#     Write-Host -ForegroundColor DarkRed "OpenSSH client does not appear to be installed. Please install or update OpenSSH to version $minimumVersion."
#     $setupSuccessful = $false
# }

# # If any software was installed, restart the script to check for the installed software
# if ($softwareInstalled) {
#     Write-Host -ForegroundColor DarkGreen "`nRestarting the script to check for the installed software..."
#     Write-Host -ForegroundColor DarkGreen "You might need to restart your computer to complete the installation of the software."
#     Start-Sleep -Seconds 5
#     # Restart the script
#     & PowerShell.exe -ExecutionPolicy Bypass -File $MyInvocation.MyCommand.Path
#     exit
# }

# Write-Host -ForegroundColor DarkGreen "`nSetting up SSH configuration..."

# # Check for SSH keys in the default location
# if (Test-Path $sshKeyPath) {
#     Write-Host "SSH keys found at $sshKeyPath"
# }
# else {
#     Write-Host -ForegroundColor DarkRed "No SSH-Keys found at $sshKeyPath."
#     # Check if user wants to add a new SSH key
#     $addNewKey = Read-Host "`nAdd a new SSH key? (yes/no)`n"
#     if ($addNewKey -eq "yes" -or $addNewKey -eq "y" -or $addNewKey -eq "Y") {
#         # Run ssh-keygen to generate a new SSH key
#         try {
#             ssh-keygen 

#             Write-Host -ForegroundColor DarkGreen "New SSH key generated successfully."
#             Write-Host -ForegroundColor DarkGreen "Please add the new SSH key to your GitLab/GitHub account and re-run the script."
#             exit 
#         }
#         catch {
#             Write-Host -ForegroundColor DarkRed "Failed to generate a new SSH key."
#             Write-Host -ForegroundColor DarkRed "Please run 'ssh-keygen' manually to generate a new SSH key."
#             $setupSuccessful = $false
#         }
#     }
#     else {
#         Write-Host -ForegroundColor DarkRed "`nPlease add an SSH key to your GitLab account and re-run the script.`n"
#         $setupSuccessful = $false
#     }
# }
# function Add-SshKeysToAgent {
#     try {
#         Write-Host "Adding SSH keys to the ssh-agent..."
#         ssh-add $sshKeyPath
#         Write-Host -ForegroundColor Green "SSH key added to the ssh-agent."
#     }
#     catch {
#         Write-Host -ForegroundColor DarkRed "Failed to add SSH key to the ssh-agent."
#     }
# }

# # Setup SSH Agent
# try {
#     $sshAgentService = Get-Service -Name ssh-agent -ErrorAction Stop
#     if ($sshAgentService.Status -ne 'Running') {
#         $startAgent = Read-Host "Start the ssh-agent service and add existing SSH keys? `nThis will start the ssh-agent on system startup, skip if you have security conserns. (yes/no)`n"
#         if ($startAgent -eq "yes" -or $startAgent -eq "y" -or $startAgent -eq "Y") {
#             Write-Host "`nStarting ssh-agent service..."
#             Set-Service ssh-agent -StartupType Automatic
#             Start-Service ssh-agent

#             if ($sshAgentService.Status -eq 'Running') {
#                 Write-Host -ForegroundColor DarkGreen "ssh-agent service started successfully."
#                 Add-SshKeysToAgent
#             }
#             else {
#                 Write-Host -ForegroundColor DarkRed "Failed to start ssh-agent service."
#                 $setupSuccessful = $false
#             }
#         }
#         else {
#             Write-Host -ForegroundColor DarkRed "Skipping ssh-agent service setup."
#             Write-Host -ForegroundColor DarkRed "You will need to add a new SSH key to the Dev Container manually before cloning the repository."
#         }
#     }
#     else {
#         Write-Host "ssh-agent already running."
#     }
# }
# catch {
#     Write-Host -ForegroundColor DarkRed "ssh-agent service not found."
#     Write-Host -ForegroundColor DarkRed "Please make sure that the latest OpenSSH Client is installed and try again."
#     $setupSuccessful = $false
# }

# # Check if any SSH keys are added to the ssh-agent
# try {
#     $null = ssh-add -l
#     $lastExitCode = $LASTEXITCODE

#     # Check the exit code of 'ssh-add -l'
#     if ($lastExitCode -eq 0) {
#         Write-Host "SSH keys have been added to the ssh-agent:"
#         ssh-add -l
#     }
#     else {
#         Write-Host -ForegroundColor DarkRed "No SSH keys have been added to the ssh-agent."
#         $addAgent = Read-Host "Add existing SSH keys to the ssh-agent? (yes/no)`n"
#         if ($addAgent -eq "yes" -or $addAgent -eq "y" -or $addAgent -eq "Y") {
#             Add-SshKeysToAgent
#         }
#         else {
#             Write-Host -ForegroundColor DarkRed "You will need to add a new SSH key to the Dev Container manually before cloning the repository."
#             $setupSuccessful = $false
#         }
#     }
# }
# catch {
#     Write-Host -ForegroundColor DarkRed "Could not check if any SSH keys have been added to the ssh-agent."
#     $setupSuccessful = $false
# }

# Write-Host -ForegroundColor DarkGreen "`nTesting SSH configuration..."

# # Test SSH connection to GitLab
# Write-Host "`Connecting to $sshTestServer..."
# try {
#     ssh -T $sshTestServer
#     Write-Host -ForegroundColor Green "SSH connection sucessful."
# }
# catch {
#     Write-Host -ForegroundColor DarkRed "Failed to establish an SSH connection to $sshTestServer."
#     $setupSuccessful = $false
# }

Write-Host -ForegroundColor DarkGreen "`nSetting up Windows Subsystem for Linux (WSL)..."

# Set WSL to use UTF-8 character encoding, cause for whatever reason UTF-16LE is the default encoding for wsl commands
$env:WSL_UTF8 = 1

# Ask the user to install Ubuntu-24.04 as the default WSL distro if the Docker Desktop distro is set as default
try {
    $wslDistroList = wsl -l --all 
    # Match all characters infront of '(Default)' in the output of 'wsl -l --all'
    $defaultDistroRegex = '(?m)^(.+?)\s+\(Default\)$'
    
    $defaultDistroLine = $wslDistroList | Select-String -Pattern $defaultDistroRegex
    if ($defaultDistroLine) {
        $currentDefaultDistro = $defaultDistroLine.Matches.Groups[1].Value

        Write-Host "Current default WSL distro: $currentDefaultDistro"
        
        # Ask the user to install Ubuntu-24.04 if the Docker Desktop distro is set as default
        if ($currentDefaultDistro -match "^docker-desktop(-data)?") {
            Write-Host -ForegroundColor DarkRed "Your current default WSL distro is set to '$currentDefaultDistro'. `nIt's necessary to use another distro for USB device management."
            $installDistro = Read-Host "`nInstall $wslDistro on WSL? (yes/no)`n"

            Write-Host "`nYou will be promted to provide a username and password for the new WSL distro."
            Write-Host "After the installation is complete you need to close the bash shell by typing 'exit'."
            Read-Host "Press Enter to continue..."

            if ($installDistro -eq "yes" -or $installDistro -eq "y" -or $installDistro -eq "Y") {
                try {
                    wsl --install -d $wslDistro
                    wsl --set-default $wslDistro
                    
                    Write-Host -ForegroundColor Green "$wslDistro installed successfully and set as default."

                    # Restart the script
                    & PowerShell.exe -ExecutionPolicy Bypass -File $MyInvocation.MyCommand.Path
                    exit
                }
                catch {
                    Write-Host -ForegroundColor DarkRed "Failed to install $wslDistro."
                    $setupSuccessful = $false
                }
            }
            else {
                Write-Host -ForegroundColor DarkRed "The docker-desktop WSL distro cannot be used for USB device management."
                Write-Host -ForegroundColor DarkRed "Please install $wslDistro or set another WSL distro as the default."
                $setupSuccessful = $false
            }
        }
    }
    else {
        Write-Host -ForegroundColor DarkRed "Could not determine the default WSL distro."
    }
}
catch {
    Write-Host -ForegroundColor DarkRed "Failed to check the default WSL distro."
}

# Change to the directory of the script 
$filePath = (Split-Path -Parent $MyInvocation.MyCommand.Path) 
Set-Location -Path $filePath  
$repoRootPath = Resolve-Path -Path "..\..\"
Set-Location -Path $repoRootPath

# Add udev rules for serial devices to the WSL distro
Write-Host -ForegroundColor DarkGreen "`nSetting up udev rules for serial devices..."
try {
    wsl bash -c "$udevBashScriptPath"
}
catch {
    Write-Host -ForegroundColor DarkRed "Failed to add udev rules on the WSL."
    $setupSuccessful = $false
}

# Check if the setup was successful
if ($setupSuccessful) {
    Write-Host -ForegroundColor DarkGreen "`nDev Container setup completed successfully."

    # Start VS Code if the setup was successful
    Write-Host -ForegroundColor DarkGreen "Starting VS Code..."
    Start-Sleep -Seconds 5
    
    # Open the project in a new VS Code window 
    code . --wait --new-window --goto .\README.md
}
else {
    Write-Host -ForegroundColor DarkRed "`nDev Container setup failed. Please check the error messages and try again."
}

