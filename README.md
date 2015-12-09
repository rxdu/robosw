# SRCL Control

This repository contains a collection of software that are used to develop and test robot related algorithms with physics-based simulation at SRCL.

## 1. Repository structure

+ control : control code for the robot/simulation
  - hummingbird
  - rc_car
+ planning : planning algorithms
  - octomap
  - quadtree
+ (build) : default location to build the code in planning folder, not tracked in git

## 2. Use Eclipse to build project

* Create a new folder outside of the project root directory

```
$ cd ~/Workspace/srcl_robot_suite/srcl_ctrl/
$ mkdir build
$ cd build
```
* Run the command to generate eclipse project from cmake

```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../planning
```
* Import generated project located at build folder into eclipse

You can install an Eclipse plugin from the following source to edit CMAKE files:

```
Name: CMAKE Editor
Location: http://cmakeed.sourceforge.net/eclipse/
```

## [Reference]

* [Check integer is power of two](http://www.exploringbinary.com/ten-ways-to-check-if-an-integer-is-a-power-of-two-in-c/)
* [OpenCV function implementation reference](https://github.com/Itseez/opencv/blob/master/modules/imgproc/src/thresh.cpp#L1192)
