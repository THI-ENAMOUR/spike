# ENAMOUR Informatics Project

The ENAMOUR project of the Technische Hochschule Ingolstadt focuses on controlling a robot with voice commands while it simulates the typical behaviour and emotions of a robot dog. For this purpose this repository contains the source code for controlling a A1 UNITREE robot. It receives data from other teams of this project and moves the robot accordingly. 

## Quick Links
- [Google Drive](https://drive.google.com/drive/folders/1B8qi29FghHsNl_soR5X4cqz8EOtiM_Sc)
- [Scrum Board](https://trello.com/b/5Z5Lfzrq/enamour-informatik)

## Quick Start

### Building the project
In order to build the project catkin is required. Executing `source make.sh` builds all modules under the "src/" dir and updates your environment with required variables. The script for setting up your environment can be found under "scripts/setup_environment.sh". Prefferabily, execute all commands while being sudo, since ROS uses system calls. Use `sudo -s` to switch into sudo mode, then execute `source make.sh` again.

### Try it out
To test the project:
- Build the project with `source make.sh`
- Execute `roslaunch champ_config bringup.launch rviz:=true`
- In another terminal (CTRL + SHIFT + T) execute `roslaunch champ_teleop teleop.launch`.

## Technologies used and How-To-Dos

### Git
Git is a distributed version control system to enable collaboration between our team members.

#### Setting up your git config
Git needs to know who the author of a commit is. Therefore you need to execute `./scripts/setup_git.sh "THI-LOGIN"` with your own thi login name. 

#### Most important commands
- To create a new branch: `git checkout -b BRANCH-NAME` 
  - New software functionalities should be in a branch startin with feature/BRANCH-NAME
- To switch to a branch: `git checkout BRANCH-NAME`
- To see the status of your local changes: `git status`
- To add all changed files to your commit (in the root dir of the project): `git add .`
- To create a local commit with your added files: `git commit -m "YOUR COMMIT MESSAGE"`
- To push your local commits: `git push`
- To squash multiple commits into one use [this guide](https://medium.com/@slamflipstrom/a-beginners-guide-to-squashing-commits-with-git-rebase-8185cf6e62ec)
  - *ONLY DO THIS IF YOU WORK ALONE ON YOUR OWN FEATURE BRANCH!*
  - Use this method before creating a pull request in order to reduce unnecessary commit messages
  - You need to use `git push --force` since you change the git history
  - If you dont know how to use the editor (vim), then take a look [here](https://eastmanreference.com/a-quick-start-guide-for-beginners-to-the-vim-text-editor)
  - You can change the default editor for git commands, e. g. a [quick guide](https://stackoverflow.com/a/36644561) for setting vscode as your editor
- To create a pull request use the [web gui](https://github.com/THI-ENAMOUR/spike/compare)

### UNITREE A1
This is the robot currently used. Everything code related is in their [offical github account](https://github.com/unitreerobotics)-

### ROS
The Robot Operating System is used as a basis to send data between system nodes. Offical documenation can be found [here](https://wiki.ros.org/Documentation).

### Champ
This framework is utilized ontop of ROS in order to reduce boiler plate and help us creating a basic architecture. Offical repository can be found [here](https://github.com/chvmp/champ).

### Simulation
For simulation mainly [Gazebo](https://gazebosim.org/) and [RVIZ](https://wiki.ros.org/rviz) are used.
