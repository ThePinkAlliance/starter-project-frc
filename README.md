# Starter FRC Project

## Getting Started

Cloning the project requires a special flag because of the included libraries (core, swerve-lib), In order to do this use the `--recurse-submodule` flag when cloning.

```shell
git clone https://github.com/ThePinkAlliance/Starter-Project-FRC.git --recurse-submodules
```

## Changing project git origin

For most projects using git you probably have git project setup on a hosting site like Github, Gitlab, Bitbucket, etc. so before writing some code you want to change the local git origin url to the one of your project on a hosting site, why do we need to do this? it's because if your using this project as a template for your frc project when pushing changes to your hosting site of choice you don't be able to because it will be pointing to our repo so in order to get the project pointing to your repo you need to enter the following into your vscode project terminal.

```shell
git remote set-url origin <your-url>
```

Just in case if your confused on what the url for your project might be, visit its page on your hosting site of choice and copy and paste that url and replace `<your-url>` with it.
