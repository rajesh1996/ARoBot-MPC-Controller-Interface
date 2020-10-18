## ARoBot - A PID controller Interface module for Ackerman Steering based Autonmous Robots
[![Build Status](https://travis-ci.org/rajesh1996/ARoBot-MPC-Controller-Interface.svg?branch=master)](https://travis-ci.org/rajesh1996/ARoBot-MPC-Controller-Interface)
[![Coverage Status](https://coveralls.io/repos/github/rajesh1996/ARoBot-MPC-Controller-Interface/badge.svg?branch=master)](https://coveralls.io/github/rajesh1996/ARoBot-MPC-Controller-Interface?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Authors
* Rajeshwar N S  - Sprint 1 (Driver) Sprint 2 (Navigator)
* Arjun Srinivasan Ambalam - Sprint 1 (Navigator) Sprint 2 (Driver)

## Overview
A controller interface for an Ackerman Kinematic model which continiuosly predicts the steering angle and wheel velocities

## Agile Iterative Process Log sheet

[Agile log sheet](https://drive.google.com/file/d/16rTux2BRGNPGFXxO9nNet1ViL05aODM7/view?usp=sharing)


## Standard install via command-line
```
git clone --recursive https://github.com/rajesh1996/ARoBot-MPC-Controller-Interface 
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## License
This software has been lincesed under [BSD 3-Clause](https://github.com/rajesh1996/ARoBot-MPC-Controller-Interface/blob/master/LICENSE.md)
