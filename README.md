# FRC-2022

Iron Panthers repository for 2022 season code.

## Setting up

### Before working: make sure you have the 2022 tools:

- WPILib: download the latest release from [allwpilib](https://github.com/wpilibsuite/allwpilib/tags). Run installer executable. Should include VS Code + extensions
  - using provided vscode is easier, but not ideal for everyone - it is optional
  - if you want to use your own vscode, just follow steps below to get extension running
- if you want to install VS Code extension separately or for your own vscode instance
  - download the latest .vsix release from [vscode-wpilib](https://github.com/wpilibsuite/vscode-wpilib/tags)
  - open vscode
  - trigger command, which is ``ctrl shift p`` on linux
  - type install from and select vsix
  - select the vsix you downloaded

### Run a test build

Open command pallette (``ctrl shift p`` on linux) and type "build robot code" until you see the command. Running this command will build ***but not deploy*** your robot code, so you can check for syntax, linting, and library issues, among other things. If this says ``BUILD SUCCESSFUL`` it means all is working.

### Install additional recommended extensions

You should install a few vscode extensions to make our lives easier.

- Spotless Gradle
  - this will let you see linting errors in the editor
  - this will also automatically format your code to conform to linting rules
- Code Spell Checker (cSpell)
  - this will highlight likely typos
  - if you see something highlighted as a typo when it shouldn't be, like a robotics term, use the code action to ``Add: "xxx" to workspace settings`` so we all can benefit

## Getting work done

### Building to test code

If you want to build code and run tests, the build robot code command will do that. However, the vscode test task is also configured to do this, so running that with the keybinding you set will have the same effect.

To configure keybindings for testing and deploying, open the command pallette and run open keyboard shortcuts. Search ``tasks: run build`` to see build, and ``tasks: run test`` to see test. I configured my test task to be ``ctrl shift b ctrl shift j`` and my build task to be ``ctrl shift b ctrl shift b``. Vscode keybindings support chording, so you can put many things as the second layer of a binding.

### Deploying code

1. Turn on the robot
