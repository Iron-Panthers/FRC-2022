# FRC-2022

Iron Panthers repository for 2022 season code.

## Setting up

### Before working: make sure you have the 2022 tools:
- WPILib: download the latest release from https://github.com/wpilibsuite/allwpilib/tags. Run installer executable. Should include VS Code + extensions
  - using provided vscode is easier, but not ideal for everyone - it is optional
  - if you want to use your own vscode, just follow steps below to get extension running
- if you want to install VS Code extension separately or for your own vscode instance
  - download the latest .vsix release from https://github.com/wpilibsuite/vscode-wpilib/tags
  - open vscode
  - trigger command, which is ``ctrl shift p`` on linux
  - type install from and select vsix
  - select the vsix you downloaded
### Run a test build
Open command pallette (``ctrl shift p`` on linux) and type "build robot code" until you see the command. Running this command will build ***but not deploy*** your robot code, so you can check for syntax, linting, and library issues, among other things.
